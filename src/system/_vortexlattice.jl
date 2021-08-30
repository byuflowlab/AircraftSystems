#=##############################################################################################
Filename: vortexlattice.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file is meant to provide convenience functions for building VortexLattice objects.
        This includes convenience constructors for common wing configurations.
=###############################################################################################

"""
    VortexLatticeSystem{T1, T2, T3, T4}

Defines a wing system for use with VortexLattice.jl.

# Fields:

* `system::VortexLattice.System` : a VortexLattice System struct
* `lifting_line_rs`
* `lifting_line_chords`
* `grids::Array{Array{Float64,2}}` : a vector of 2-D arrays, each 2-D array containing wing panel corner locations; returned by `VortexLattice.wing_to_surface_panels()`

"""
struct VortexLatticeSystem{TF}
    system::VL.System{TF}
    lifting_line_rs::Vector{Array{TF,2}}
    lifting_line_chords::Vector{Vector{TF}}
    cmacs::Vector{TF}
    airfoil_polars
end

"""
Constructor for `VortexLatticeSystem`.

# Arguments:

* `lexs::Array{Array{Float64,1}}` : array of vectors of x coordinates of the leading edge
* `leys::Array{Array{Float64,1}}` : array of vectors of y coordinates of the leading edge
* `lezs::Array{Array{Float64,1}}` : array of vectors of z coordinates of the leading edge
* `chords::Array{Array{Float64,1}}` : array of vectors of chord lengths
* `thetas::Array{Array{Float64,1}}` : array of vectors of twist angles in radians
* `phis::Array{Array{Float64,1}}` : array of vectors of dihedral angles in radians
* `nspanwisepanels::Array{Float64,1}` : vector of the number of desired spanwise panels
* `nchordwisepanels::Array{Float64,1}` : vector of the number of desired chordwise panels

# Keyword Arguments:

* `wing_cambers = [fill((xc) -> 0, length(lex)) for lex in lexs]` : vector of vectors of functions describing the camber line
* `spanwise_spacings = fill(VL.Uniform(), length(lexs))` : vector of `<: VortexLattice.AbstractSpacing` objects describing the spanwise spacing
* `chordwise_spacings = fill(VL.Uniform(), length(lexs))` : vector of `<: VortexLattice.AbstractSpacing` objects describing the chordwise spacing
* `Vref = 1.0` : velocity used to normalize
* `symmetric = true` : whether or not to mirror the geometry about the x-z plane
* `iref = 1` : index of `lexs`, `leys`, etc. to use when calculating reference properties
* `static_margin = 0.10` : static margin of the `iref`th lifting surface

# Returns:

* `vlmsystem::VortexLatticeSystem`

"""
function VortexLatticeSystem(lexs, leys, lezs, chords, thetas, phis, nspanwisepanels, nchordwisepanels;
            cambers = [fill((xc) -> 0, length(lex)) for lex in lexs],
            spanwise_spacings = fill(VL.Uniform(), length(lexs)),
            chordwise_spacings = fill(VL.Uniform(), length(lexs)),
            Vref = 1.0, symmetric = true, iref = 1, static_margin = 0.10, liftingline_x_over_c = 0.25
            ) where T <: AbstractArray

    @assert typeof(lexs[1]) <: AbstractArray "lexs must be a vector of vectors"
    @assert size(lexs) == size(leys) && size(lexs[1]) == size(leys[1])   "size of leading edge xs and ys not consistent"
    @assert size(lexs) == size(lezs) && size(lexs[1]) == size(lezs[1])   "size of leading edge xs and zs not consistent"
    @assert size(lexs) == size(chords) && size(lexs[1]) == size(chords[1]) "size of leading edge xs and chords not consistent"
    @assert size(lexs) == size(thetas) && size(lexs[1]) == size(thetas[1]) "size of leading edge xs and thetas not consistent"
    @assert size(lexs) == size(phis) && size(lexs[1]) == size(phis[1]) "size of leading edge xs and phis not consistent"

    nwings = length(lexs)

    # build grids and surfaces
    gridssurfaces = [VL.wing_to_surface_panels(lexs[i], leys[i], lezs[i], chords[i], thetas[i], phis[i], nspanwisepanels[i], nchordwisepanels[i];
        fc = cambers[i], spacing_s=spanwise_spacings[i], spacing_c=chordwise_spacings[i]) for i in 1:nwings]

    # extract grids and surfaces
    grids = [gridsurface[1] for gridsurface in gridssurfaces]
    surfaces = [gridsurface[2] for gridsurface in gridssurfaces]

    # build reference
    ref = get_reference(lexs[iref], leys[iref], chords[iref], static_margin, Vref; symmetric)

    # build freestream
    alpha = 3.0 * pi/180
    beta = 0.0
    Omegas = StaticArrays.@SVector zeros(3)
    vlmfreestream = VL.Freestream(Vref, alpha, beta, Omegas)

    # get lifting line geometry
    rs, chords = VL.lifting_line_geometry(grids)
    cmacs = [FM.trapz(r[2,:], chords[i]) / (r[2,end] - r[2,1]) for (i,r) in enumerate(rs)]
    # build system
    system = VL.steady_analysis(surfaces, ref, vlmfreestream; symmetric)
    vlmsystem = VortexLatticeSystem(system, rs, chords, cmacs, nothing)

    return vlmsystem
end

"""
    get_reference(lex, ley, chord, static_margin, Vref)

# Arguments:

* `lex`
* `ley`
* `chord`
* `static_margin`
* `Vref`

"""
function get_reference(lex, ley, chord, static_margin, Vref; symmetric = true)

    bref = ley[end] * 2
    Sref = FM.trapz(ley, chord) # area of starboard wing only
    if symmetric; Sref *= 2; end
    cref = Sref / bref
    lex_midpoint = get_midpoints(lex)
    dy = [ley[i+1] - ley[i] for i in 1:length(ley)-1]
    xmean = weightedmean(lex_midpoint, dy) # might be worth testing
    rref = [xmean + cref * (0.25 - static_margin), 0.0, 0.0]
    Vinf = Vref
    ref = VL.Reference(Sref, cref, bref, rref, Vinf)

    return ref
end

weightedmean(set, weights) = sum(set .* weights) / sum(weights)

get_midpoints(a) = [(a[i] + a[i+1])/2 for i in 1:length(a)-1]

"""
    simplewing_system(b, TR, AR, θroot, θtip, le_sweep, ϕ;
        xle=[0.0, 0.0], yle=[0.0, b/2], zle=[0.0, 0.0],
        cmac=b / AR, # AR = b/c => c = b/AR
        Sref=b * cmac,
        chord=[cmac * 2 / (1 + TR), cmac * 2 / (1/TR + 1)], twist=[θroot, θtip], phi=[0.0,0.0],
        static_margin=0.10,
        Vinf=1.0,
        Vref=Vinf,
        alpha=0.0,
        beta=0.0,
        Omega=[0.0, 0.0, 0.0],
        camber=fill((xc) -> 0, 2),
        npanels=50, nchordwisepanels=1,
        symmetric=true,
        spacing_s = symmetric ? VL.Cosine() : VL.Sine(),
        spacing_c=VL.Uniform(),
        mirror=!symmetric,
        kwargs...
    )

Convenience constructor for a single wing `VortexLatticeSystem`.

# Keyword Arguments:

* `wing_b`: wing span
* `wing_TR`: wing taper ratio
* `wing_AR`: wing aspect ratio
* `wing_θroot`: wing root twist (radians)
* `wing_θtip`: wing tip twist (radians)
* `xle`: leading-edge x-coordinates
* `yle`: leading-edge y-coordinates
* `zle`: leading-edge z-coordinates
* `cmac`: mean aerodynamic chord
* `Sref`: reference wing area
* `wing_chord`: chord at each wing section
* `wing_twist`: twist at each wing section
* `wing_phi`: wing section rotation about the x-axis
* `static_margin`: static margin of the wing
* `Vinf`: freestream velocity
* `Vref`: reference velocity; defaults to freestream velocity
* `alpha`: angle of attack (radians)
* `beta`: sideslip angle (radians)
* `Omega`: rotational velocity around the reference location
* `wing_camber`: camberline function for each section
* `wing_npanels`: number of spanwise panels for the wing
* `wing_nchordwisepanels`: number of chordwise panels for the wing
* `symmetric::Bool`: Flag indicating whether a mirror image across the X-Z plane should be used when calculating induced velocities. Defaults to true. From my understanding, this solves the half span and uses a mirrored aproach for induced velocities, whereas mirror will physically mirror the geometry for you and solve it all together (Tyler).
* `mirror::Bool`: mirrors the geometry over the x-z plane
* `wing_spacing_s::Int`: spanwise panel spacing scheme for the wing; defaults to Cosine (or Sine if mirror==true, which results in Cosine anyway)
* `wing_spacing_c::Int`: chordwise panel spacing scheme for the wing; defaults to Uniform()

"""
function simplewing_system(b, TR, AR, θroot, θtip, le_sweep, ϕ;
    yle = [0.0,b/2], xle = [0.0, yle[2] * tan(le_sweep)], zle = [0.0, yle[2] * tan(ϕ)],
    cmac = b/AR, # AR = b/c => c = b/AR
    Sref = b*cmac,
    chord = [cmac * 2 / (1 + TR), cmac * 2 / (1 + 1/TR)], twist=[θroot, θtip], phi=[ϕ, ϕ],
    static_margin = 0.10,
    Vref = 1.0, # in case you want the reference velocity to be different than the freestream velocity
    camber = fill((xc) -> 0, length(xle)),
    npanels = 50, nchordwisepanels=1,
    symmetric = true, # if true, we don't need to mirror geometry. Good option if parameters/geometry is symmetric and you don't care about lateral stability derivatives
    mirror = !symmetric,
    spacing_s = symmetric ? VL.Cosine() : VL.Sine(), # if mirrored, sine spacing becomes cosine spacing
    spacing_c = VL.Uniform(), # should only ever need uniform chordwise spacing
    kwargs...
)

    Vinf = 1.0
    alpha=0.0
    beta=0.0
    Omega=StaticArrays.@SVector [0.0,0.0,0.0]

    #=
    cr + ct = cmac * 2
    ct = λ * cr
    ct = cmac * 2 - cr
    λ * cr = cmac * 2 - cr
    cr = 2 * cmac / (1 + λ)

    cr = cmac * 2 - ct
    cr = ct / λ
    cmac * 2 - ct = ct / λ
    cmac * 2 = ct * (1 + 1/λ)
    ct = cmac * 2 / (1 + 1/λ)
    =#

    # Check for errors
    @assert size(xle) == size(yle)    "size of leading edge xs and ys not consistent"
    @assert size(yle) == size(zle)    "size of leading edge ys and zs not consistent"
    @assert size(xle) == size(chord)    "size of leading edge xs and chord not consistent"
    @assert size(xle) == size(twist)    "size of leading edge xs and twist not consistent"
    @assert size(xle) == size(phi)    "size of leading edge xs and phi not consistent"
    @assert yle[1] == 0.0    "yle must begin at the root (0.0)"
    @assert yle[end] == b/2    "yle must end at the tip (span/2)"
    @assert !(symmetric && mirror)    "symmetric and mirror cannot both be true"

    # construct surfaces and grids
    grid, surface = VL.wing_to_surface_panels(xle, yle, zle, chord, twist, phi, npanels, nchordwisepanels;
                        fc = camber, spacing_s, spacing_c, mirror)

    grids = [grid]
    surfaces = [surface]

    # reference parameters
    cgx = cmac * (0.25 - static_margin)
    rref = [cgx, 0.0, 0.0]
    reference = VL.Reference(Sref, cmac, b, rref, Vref)

    # initialize dummy freestream parameters
    fs = VL.Freestream(Vinf, alpha, beta, Omega)

    # perform steady state analysis
    vlmsystem = VL.steady_analysis(surfaces, reference, fs; symmetric)
    lifting_line_rs, lifting_line_chords = VL.lifting_line_geometry(grids)
    cmacs = [FM.trapz(lifting_line_r[2,:], lifting_line_chords[i]) / (lifting_line_r[end] - lifting_line_r[1]) for (i,lifting_line_r) in enumerate(lifting_line_rs)]
    wing_system = VortexLatticeSystem(vlmsystem, lifting_line_rs, lifting_line_chords, cmacs, nothing)

    return wing_system
end


"""
    simpleairplanesystem(;
        # wing geometry
        wing_b=2.0, wing_TR=0.8, wing_AR=8.0, wing_θroot=0.0, wing_θtip=0.0,
        wing_xle=[0.0,0.0], wing_yle=[0.0, wing_b/2], wing_zle=[0.0,0.0],
        wing_cmac=wing_b/wing_AR, # AR = b/c => c = b/AR
        wing_chord=[wing_cmac * 2 / (1 + wing_TR), wing_cmac * 2 / (1/wing_TR + 1)],
        wing_theta=[wing_θroot, wing_θtip],
        wing_phi=[1.0,1.0] * 3pi/180.0,
        wing_camber=fill((xc) -> 0, length(wing_xle)),
        # tail geometry
        tail_b=0.4, tail_TR=0.7, tail_AR=6.0, tail_θroot=0.0, tail_θtip=0.0,
        tail_incidence=3pi/180, tail_position=[1.5,0.0,0.1], tail_cmac=tail_b/tail_AR,
        tail_xle=[tail_position[1],tail_position[1]],
        tail_yle=[0.0,tail_b/2],
        tail_zle=[tail_position[3],tail_position[3]],
        tail_chord=[tail_cmac*2/(1+tail_TR),tail_cmac*2/(1/tail_TR+1)],
        tail_twist=[tail_θroot, tail_θtip] .- tail_incidence,
        tail_phi=[0.0,0.0],
        tail_camber=fill((xc) -> 0, length(tail_xle)),
        # vertical tail geometry
        vertical_tail_b=0.4, vertical_tail_TR=0.7, vertical_tail_AR=6.0, vertical_tail_θroot=0.0, vertical_tail_θtip=0.0,
        vertical_tail_cmac=vertical_tail_b/vertical_tail_AR,
        vertical_tail_xle=[tail_position[1], tail_position[1]],
        vertical_tail_yle=[0.0,0.0],
        vertical_tail_zle=[tail_position[3], tail_position[3] + vertical_tail_b],
        vertical_tail_chord=[cmac_vertical_tail * 2 / (1 + vertical_tail_TR), cmac_vertical_tail * 2 / (1/vertical_tail_TR + 1)],
        vertical_tail_theta=[vertical_tail_θroot, vertical_tail_θtip],
        vertical_tail_phi=[0.0, 0.0],
        vertical_tail_camber=fill((xc) -> 0, length(vertical_tail_xle)),
        # other parameters/reference values
        Sref=wing_b*cmac,
        static_margin=0.10,
        Vinf=1.0,
        Vref=Vinf,
        alpha=0.0,
        beta=0.0,
        Omega=@SVector [0.0,0.0,0.0]
        symmetric=[true, true, false],
        mirror=[false, false, false],
        # panel setup
        wing_npanels=50, wing_nchordwisepanels=3,
        wing_spacing_s= mirror[1] ? VL.Sine() : VL.Cosine(), # if mirrored, Sine becomes Cosine
        wing_spacing_c=VL.Uniform(),
        tail_npanels=20, tail_nchordwisepanels=1,
        tail_spacing_s= mirror[2] ? VL.Sine() : VL.Cosine(),
        tail_spacing_c=VL.Uniform(),
        vertical_tail_npanels=20, vertical_tail_nchordwisepanels=1,
        vertical_tail_spacing_s= mirror[3] ? VL.Sine() : VL.Cosine(),
        vertical_tail_spacing_c=VL.Uniform(),
        kwargs...)

Convenience constructor for a simple airplane `VortexLatticeSystem`.

# Keyword Arguments:

* `wing_b`: wing span
* `wing_TR`: wing taper ratio
* `wing_AR`: wing aspect ratio
* `wing_θroot`: wing root twist
* `wing_θtip`: wing tip twist
* `wing_xle`: x-coordinates of wing leading edge
* `wing_yle`: y-coordinates of wing leading edge
* `wing_zle`: z-coordinates of wing leading edge
* `wing_cmac`: wing mean aerodynamic chord
* `wing_chord`: chord at each wing section
* `wing_theta`: twist (radians) at each wing section
* `wing_phi`: wing dihedral angle
* `wing_camber`: camberline function for each section

* `tail_b`: tail span
* `tail_TR`: tail taper ratio
* `tail_AR`: tail aspect ratio
* `tail_θroot`: tail root twist
* `tail_θtip`: tail tip twist
* `tail_incidence`: tail incidence angle (subtracts from twist)
* `tail_position`: position of the y=0 leading edge point of the tail
* `tail_cmac`: tail mean aerodynamic chord
* `tail_xle`: x-coordinates of tail leading edge
* `tail_yle`: y-coordinates of tail leading edge
* `tail_zle`: z-coordinates of tail leading edge
* `tail_chord`: chord at each tail section
* `tail_theta`: twist (radians) at each tail section
* `tail_phi`: tail dihedral angle
* `tail_camber`: camberline function for each section

* `vertical_tail_b`: vertical tail span
* `vertical_tail_TR`: vertical tail taper ratio
* `vertical_tail_AR`: vertical tail aspect ratio
* `vertical_tail_θroot`: vertical tail root twist
* `vertical_tail_θtip`: vertical tail tip twist
* `vertical_tail_cmac`: vertical tail mean aerodynamic chord
* `vertical_tail_xle`: x-coordinates of vertical tail leading edge
* `vertical_tail_yle`: y-coordinates of vertical tail leading edge
* `vertical_tail_zle`: z-coordinates of vertical tail leading edge
* `vertical_tail_chord`: chord at each vertical tail section
* `vertical_tail_theta`: twist (radians) at each vertical tail section
* `vertical_tail_phi`: vertical tail dihedral angle
* `vertical_tail_camber`: camberline function for each section

* `Sref`: wing reference area
* `static_margin`: wing static margin
* `Vinf`: freestream velocity
* `Vref`: reference velocity; defaults to freestream velocity
* `alpha`: wing angle of attack (radians)
* `beta`: wing slipstream angle (radians)
* `Omega`: rotational velocity around the reference location
* `symmetric`: uses a mirror image when calculating induced velocities
* `mirror`: adds mirrored image to the geometry and solves without symmetry

* `wing_npanels`: number of spanwise panels for the wing
* `wing_nchordwisepanels`: number of chordwise panels for the wing
* `wing_spacing_s`: spanwise panel spacing scheme for the wing; defaults to Cosine()
* `wing_spacing_c`: chordwise panel spacing scheme for the wing; defaults to Uniform()
* `tail_npanels`: number of spanwise panels for the tail
* `tail_nchordwisepanels`: number of chordwise panels for the tail
* `tail_spacing_s`: spanwise panel spacing scheme for the tail; defaults to Cosine()
* `tail_spacing_c`: chordwise panel spacing scheme for the tail
* `vertical_tail_npanels`: number of spanwise panels for the vertical tail
* `vertical_tail_nchordwisepanels`: number of chordwise panels for the vertical tail
* `vertical_tail_spacing_s`: spanwise panel spacing scheme for the vertical tail
* `vertical_tail_spacing_c`: chordwise panel spacing scheme for the vertical tail

"""
function simpleairplanesystem(;
            # wing geometry
            wing_b=2.0, wing_TR=0.8, wing_AR=8.0, wing_θroot=0.0, wing_θtip=0.0,
            wing_xle=[0.0,0.0], wing_yle=[0.0, wing_b/2], wing_zle=[0.0,0.0],
            wing_cmac=wing_b/wing_AR, # AR = b/c => c = b/AR
            wing_chord=[wing_cmac * 2 / (1 + wing_TR), wing_cmac * 2 / (1/wing_TR + 1)],
            wing_theta=[wing_θroot, wing_θtip],
            wing_phi=[1.0,1.0] * 3pi/180.0,
            wing_camber=fill((xc) -> 0, length(wing_xle)),
            # tail geometry
            tail_b=0.4, tail_TR=0.7, tail_AR=6.0, tail_θroot=0.0, tail_θtip=0.0,
            tail_incidence=3pi/180, tail_position=[1.5,0.0,0.1], tail_cmac=tail_b/tail_AR,
            tail_xle=[tail_position[1],tail_position[1]],
            tail_yle=[0.0,tail_b/2],
            tail_zle=[tail_position[3],tail_position[3]],
            tail_chord=[tail_cmac*2/(1+tail_TR),tail_cmac*2/(1/tail_TR+1)],
            tail_twist=[tail_θroot, tail_θtip] .- tail_incidence,
            tail_phi=[0.0,0.0],
            tail_camber=fill((xc) -> 0, length(tail_xle)),
            # vertical tail geometry
            vertical_tail_b=0.4, vertical_tail_TR=0.7, vertical_tail_AR=6.0, vertical_tail_θroot=0.0, vertical_tail_θtip=0.0,
            vertical_tail_cmac=vertical_tail_b/vertical_tail_AR,
            vertical_tail_xle=[tail_position[1], tail_position[1]],
            vertical_tail_yle=[0.0,0.0],
            vertical_tail_zle=[tail_position[3], tail_position[3] + vertical_tail_b],
            vertical_tail_chord=[cmac_vertical_tail * 2 / (1 + vertical_tail_TR), cmac_vertical_tail * 2 / (1/vertical_tail_TR + 1)],
            vertical_tail_theta=[vertical_tail_θroot, vertical_tail_θtip],
            vertical_tail_phi=[0.0, 0.0],
            vertical_tail_camber=fill((xc) -> 0, length(vertical_tail_xle)),
            # other parameters/reference values
            Sref=wing_b*cmac,
            static_margin=0.10,
            Vinf=1.0,
            Vref=Vinf,
            alpha=0.0,
            beta=0.0,
            Omega=(StaticArrays.@SVector [0.0,0.0,0.0]),
            symmetric=[true, true, false],
            mirror=[false, false, false],
            # panel setup
            wing_npanels=50, wing_nchordwisepanels=3,
            wing_spacing_s= mirror[1] ? VL.Sine() : VL.Cosine(), # if mirrored, Sine becomes Cosine
            wing_spacing_c=VL.Uniform(),
            tail_npanels=20, tail_nchordwisepanels=1,
            tail_spacing_s= mirror[2] ? VL.Sine() : VL.Cosine(),
            tail_spacing_c=VL.Uniform(),
            vertical_tail_npanels=20, vertical_tail_nchordwisepanels=1,
            vertical_tail_spacing_s= mirror[3] ? VL.Sine() : VL.Cosine(),
            vertical_tail_spacing_c=VL.Uniform(),
            kwargs...)

    # Check for errors
    @assert size(wing_xle) == size(wing_yle)    "size of leading edge xs and ys not consistent for the wing"
    @assert size(wing_yle) == size(wing_zle)    "size of leading edge ys and zs not consistent for the wing"
    @assert size(wing_xle) == size(wing_chord)    "size of leading edge xs and wing_chord not consistent"
    @assert size(wing_xle) == size(wing_twist)    "size of leading edge xs and wing_twist not consistent"
    @assert size(wing_xle) == size(wing_phi)    "size of leading edge xs and wing_phi not consistent"
    @assert wing_yle[1] == 0.0    "wing yle must begin at the root (0.0)"
    @assert wing_yle[end] == wing_b/2    "wing yle must end at the tip (span/2)"

    @assert size(tail_xle) == size(tail_yle)    "size of leading edge xs and ys not consistent for the tail"
    @assert size(tail_yle) == size(tail_zle)    "size of leading edge ys and zs not consistent for the tail"
    @assert size(tail_xle) == size(tail_chord)    "size of leading edge xs and tail_chord not consistent"
    @assert size(tail_xle) == size(tail_twist)    "size of leading edge xs and tail_twist not consistent"
    @assert size(tail_xle) == size(tail_phi)    "size of leading edge xs and tail_phi not consistent"
    @assert tail_yle[1] == 0.0    "tail yle must begin at the root (0.0)"
    @assert tail_yle[end] == tail_b/2    "tail yle must end at the tip (span/2)"

    @assert size(vertical_tail_xle) == size(vertical_tail_yle)    "size of leading edge xs and ys not consistent for the vertical tail"
    @assert size(vertical_tail_yle) == size(vertical_tail_zle)    "size of leading edge ys and zs not consistent for the vertical tail"
    @assert size(vertical_tail_xle) == size(vertical_tail_chord)    "size of leading edge xs and vertical_tail_chord not consistent"
    @assert size(vertical_tail_xle) == size(vertical_tail_twist)    "size of leading edge xs and vertical_tail_twist not consistent"
    @assert size(vertical_tail_xle) == size(vertical_tail_phi)    "size of leading edge xs and vertical_tail_phi not consistent"
    @assert vertical_tail_yle[1] == 0.0    "vertical tail yle must begin at the root (0.0)"

    @assert !(symmetric[1] && mirror[1])    "symmetric and mirror cannot both be true for the wing"
    @assert !(symmetric[1] && mirror[2])    "symmetric and mirror cannot both be true for the tail"
    @assert !(symmetric[1] && mirror[3])    "symmetric and mirror cannot both be true for the vertical tail"

    # construct wing surfaces and grids
    wing_grid, wing_surface = VL.wing_to_surface_panels(wing_xle, wing_yle, wing_zle, wing_chord, wing_theta, wing_phi, wing_npanels, wing_nchordwisepanels;
                                    fc = wing_camber, spacing_s=wing_spacing_s, spacing_c=wing_spacing_c, mirror=mirror[1])

    # construct tail surfaces and grids
    tail_grid, tail_surface = VL.wing_to_surface_panels(tail_xle, tail_yle, tail_zle, tail_chord, tail_theta, tail_phi, tail_npanels, tail_nchordwisepanels;
                                    fc = tail_camber, spacing_s=tail_spacing_s, spacing_c=tail_spacing_c, mirror=mirror[2])

    # construct vertical tail surfaces and grids
    vertical_tail_grid, vertical_tail_surface = VL.wing_to_surface_panels(vertical_tail_xle, vertical_tail_yle, vertical_tail_zle, vertical_tail_chord,
                                                    vertical_tail_theta, vertical_tail_phi, vertical_tail_npanels, vertical_tail_nchordwisepanels;
                                                    fc = vertical_tail_camber, spacing_s=vertical_tail_spacing_s, spacing_c=vertical_tail_spacing_c, mirror=mirror[3])

    # assemble system
    grids = [wing_grid, tail_grid, vertical_tail_grid]
    surfaces = [wing_surface, tail_surface, vertical_tail_surface]

    # reference parameters
    cgx = cmac * (0.25 - static_margin)
    rref = [cgx, 0.0, 0.0]
    reference = VL.Reference(Sref, cmac, wing_b, rref, Vref)

    # initialize freestream parameters
    fs = VL.Freestream(Vinf, alpha, beta, Omega)

    # perform steady state analysis
    vlmsystem = VL.steady_analysis(surfaces, reference, fs; symmetric)
    wing_system = VortexLatticeSystem(vlmsystem, grids)

    return wing_system
end

function VL.Freestream(freestream::Freestream)
    VL.Freestream(freestream.vinf, freestream.alpha, freestream.beta, freestream.Omega)
end

# outputs = [Outputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), Outputs(0.24285036855335476 - 0.0im, 0.09374476338448777 - 0.0im, 0.0, 0.0, 1.2308487178526557, 0.0, 0.2772045889709332, 0.3183884946733561, 4.534990314410568 - 0.0im, 1.5050759240380058, 0.13763120546656513, 1.4099532010335012, 0.5442681845675164, 0.9835773158795993, 0.9917546651665418), Outputs(0.7217628407058008 - 0.0im, 0.21697762721530664 - 0.0im, 0.0, 0.0, 1.7326960734792525, 0.0, 0.2589657900820015, 0.245706809640596, 6.7673827545271354 - 0.0im, 1.6376114138887694, 0.05416136893272035, 1.569136116792526, 0.47171648663229143, 0.9996516983041511, 0.9998258339851752), Outputs(1.2990599048174767 - 0.0im, 0.31355858702643263 - 0.0im, 0.0, 0.0, 2.0129587976546834, 0.0, 0.22688706695390065, 0.1431274468578776, 8.953184950774602 - 0.0im, 1.3892205399175588, 0.01383177005724867, 1.3505053059805476, 0.32597614162717065, 0.9989877735121309, 0.999493758615896), Outputs(1.7619807902797593 - 0.0im, 0.3720701211173133 - 0.0im, 0.0, 0.0, 2.09694555206726, 0.0, 0.19036789138116791, 0.07972442835883256, 11.104190789025736 - 0.0im, 1.09077738226134, 0.019353098469995278, 1.067410153478332, 0.2254005419795043, 0.9960145868560558, 0.9980053040220056), Outputs(2.2711845655764025 - 0.0im, 0.4405002858849476 - 0.0im, 0.0, 0.0, 2.173383411501406, 0.0, 0.1657314413795813, 0.0556670648944034, 13.265213925277973 - 0.0im, 0.9558687456440128, 0.02470688289088402, 0.9386953965963786, 0.18206164167668523, 0.9863118571582342, 0.9931323462450683), Outputs(2.4372150640530514 - 0.0im, 0.4554901469388099 - 0.0im, 0.0, 0.0, 2.1629611788994447, 0.0, 0.15314916243164545, 0.04256438043828989, 14.343403366255124 - 0.0im, 0.8800488657666018, 0.02782677887567497, 0.8655033880045069, 0.16175358145153387, 0.9771513649187485, 0.9885096686015512), Outputs(2.6371721454533454 - 0.0im, 0.47293752842306824 - 0.0im, 0.0, 0.0, 2.1681319604225773, 0.0, 0.14389224181868238, 0.0387234776796615, 15.424864001541529 - 0.0im, 0.8574901315613467, 0.028785248170534933, 0.8445005855017522, 0.15144859631086924, 0.9608451485828865, 0.9802270903126921), Outputs(2.9035116834001253 - 0.0im, 0.5070499699969895 - 0.0im, 0.0, 0.0, 2.1978736826245666, 0.0, 0.13838725719185638, 0.03708290981589751, 16.51325595934029 - 0.0im, 0.8476745782368323, 0.029258614602765128, 0.8355344940565491, 0.14591218715080329, 0.9309406549944107, 0.9648526597332935), Outputs(2.915378995839232 - 0.0im, 0.4988635277901568 - 0.0im, 0.0, 0.0, 2.132452708206054, 0.0, 0.12893626553987916, 0.029308601308119464, 17.59133845621036 - 0.0im, 0.7995946884337315, 0.03243082128604525, 0.7887875033803128, 0.13497295451969854, 0.888830318342529, 0.9427779793474862), Outputs(2.672487776120149 - 0.0im, 0.4658323109725899 - 0.0im, 0.0, 0.0, 1.9807514705340548, 0.0, 0.11703583739096063, 0.014783645024415093, 18.662919158779463 - 0.0im, 0.7077670820565516, 0.039347704185111965, 0.6983307468035365, 0.121723672045743, 0.8261271529063575, 0.908915371696594), Outputs(2.6753732822512957 - 0.0im, 0.4714559098077924 - 0.0im, 0.0, 0.0, 1.925896978460363, 0.0, 0.11765311844886973, 0.013955301300062325, 19.763948184544113 - 0.0im, 0.702698187395941, 0.039939768326604105, 0.6931521258065394, 0.1221476899224832, 0.689154508910361, 0.8301533044627125), Outputs(2.397999681058518 - 0.0im, 0.4549131149780635 - 0.0im, 0.0, 0.0, 1.774720023483803, 0.0, 0.1305556999113481, 0.01385081781673042, 20.895071940836615 - 0.0im, 0.7020550192002749, 0.04000541332594471, 0.6908722123615864, 0.1310620817257423, 0.4256466033071096, 0.652415974135451), Outputs(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]

# function Base.getproperty(obj::Vector{Outputs{TF1,TF2} where {TF1,TF2}}, sym::Symbol)
#     return getfield.(obj, sym)
# end

# function Base.getproperty(obj::Vector{Outputs{TF1,TF2}} where TF1, sym::Symbol) where TF2
#     return getfield.(obj, sym)
# end

# function Base.getproperty(obj::Vector{Outputs{TF1,TF2}} where TF2, sym::Symbol) where TF1
#     return getfield.(obj, sym)
# end

# function Base.getproperty(obj::Vector{Outputs{TF1,Float64} where TF1}, sym::Symbol)
#     return getfield.(obj, sym)
# end

# outputs2 = Outputs{Float64,Float64}[Outputs{Float64,Float64}(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), Outputs{Float64,Float64}(0.24285036855335476, 0.09374476338448777, 0.0, 0.0, 1.2308487178526557, 0.0, 0.2772045889709332, 0.3183884946733561, 4.534990314410568, 1.5050759240380058, 0.13763120546656513, 1.4099532010335012, 0.5442681845675164, 0.9835773158795993, 0.9917546651665418), Outputs{Float64,Float64}(0.7217628407058008, 0.21697762721530664, 0.0, 0.0, 1.7326960734792525, 0.0, 0.2589657900820015, 0.245706809640596, 6.7673827545271354, 1.6376114138887694, 0.05416136893272035, 1.569136116792526, 0.47171648663229143, 0.9996516983041511, 0.9998258339851752), Outputs{Float64,Float64}(1.2990599048174767, 0.31355858702643263, 0.0, 0.0, 2.0129587976546834, 0.0, 0.22688706695390065, 0.1431274468578776, 8.953184950774602, 1.3892205399175588, 0.01383177005724867, 1.3505053059805476, 0.32597614162717065, 0.9989877735121309, 0.999493758615896), Outputs{Float64,Float64}(1.7619807902797593, 0.3720701211173133, 0.0, 0.0, 2.09694555206726, 0.0, 0.19036789138116791, 0.07972442835883256, 11.104190789025736, 1.09077738226134, 0.019353098469995278, 1.067410153478332, 0.2254005419795043, 0.9960145868560558, 0.9980053040220056), Outputs{Float64,Float64}(2.2711845655764025, 0.4405002858849476, 0.0, 0.0, 2.173383411501406, 0.0, 0.1657314413795813, 0.0556670648944034, 13.265213925277973, 0.9558687456440128, 0.02470688289088402, 0.9386953965963786, 0.18206164167668523, 0.9863118571582342, 0.9931323462450683), Outputs{Float64,Float64}(2.4372150640530514, 0.4554901469388099, 0.0, 0.0, 2.1629611788994447, 0.0, 0.15314916243164545, 0.04256438043828989, 14.343403366255124, 0.8800488657666018, 0.02782677887567497, 0.8655033880045069, 0.16175358145153387, 0.9771513649187485, 0.9885096686015512), Outputs{Float64,Float64}(2.6371721454533454, 0.47293752842306824, 0.0, 0.0, 2.1681319604225773, 0.0, 0.14389224181868238, 0.0387234776796615, 15.424864001541529, 0.8574901315613467, 0.028785248170534933, 0.8445005855017522, 0.15144859631086924, 0.9608451485828865, 0.9802270903126921), Outputs{Float64,Float64}(2.9035116834001253, 0.5070499699969895, 0.0, 0.0, 2.1978736826245666, 0.0, 0.13838725719185638, 0.03708290981589751, 16.51325595934029, 0.8476745782368323, 0.029258614602765128, 0.8355344940565491, 0.14591218715080329, 0.9309406549944107, 0.9648526597332935), Outputs{Float64,Float64}(2.915378995839232, 0.4988635277901568, 0.0, 0.0, 2.132452708206054, 0.0, 0.12893626553987916, 0.029308601308119464, 17.59133845621036, 0.7995946884337315, 0.03243082128604525, 0.7887875033803128, 0.13497295451969854, 0.888830318342529, 0.9427779793474862), Outputs{Float64,Float64}(2.672487776120149, 0.4658323109725899, 0.0, 0.0, 1.9807514705340548, 0.0, 0.11703583739096063, 0.014783645024415093, 18.662919158779463, 0.7077670820565516, 0.039347704185111965, 0.6983307468035365, 0.121723672045743, 0.8261271529063575, 0.908915371696594), Outputs{Float64,Float64}(2.6753732822512957, 0.4714559098077924, 0.0, 0.0, 1.925896978460363, 0.0, 0.11765311844886973, 0.013955301300062325, 19.763948184544113, 0.702698187395941, 0.039939768326604105, 0.6931521258065394, 0.1221476899224832, 0.689154508910361, 0.8301533044627125), Outputs{Float64,Float64}(2.397999681058518, 0.4549131149780635, 0.0, 0.0, 1.774720023483803, 0.0, 0.1305556999113481, 0.01385081781673042, 20.895071940836615, 0.7020550192002749, 0.04000541332594471, 0.6908722123615864, 0.1310620817257423, 0.4256466033071096, 0.652415974135451), Outputs{Float64,Float64}(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]

# struct Outputs{TF1,TF2}
#     Np::TF1
#     Tp::TF1
#     a::TF2
#     ap::TF2
#     u::TF2
#     v::TF2
#     phi::TF2
#     alpha::TF2
#     W::TF1
#     cl::TF2
#     cd::TF2
#     cn::TF2
#     ct::TF2
#     F::TF2
#     G::TF2
# end

# function Base.getproperty(obj::Vector{Outputs{TF1,TF2} where {TF1,TF2}}, sym::Symbol)
#     println("Sherlock! Method 1")
#     return getfield.(obj, sym)
# end

# function Base.getproperty(obj::Vector{Outputs{TF1,TF2}}, sym::Symbol) where {TF1,TF2}
#     println("Sherlock! Method 2")
#     return getfield.(obj, sym)
# end

# function Base.getproperty(obj::Vector{Outputs{TF1,TF2} where TF2}, sym::Symbol) where TF1
#     println("Sherlock! Method 3")
#     return getfield.(obj, sym)
# end

# function Base.getproperty(obj::Vector{Outputs{TF1,TF2} where TF1}, sym::Symbol) where TF2
#     println("Sherlock! Method 4")
#     return getfield.(obj, sym)
# end

# function Base.getproperty(obj::Vector{Outputs}, sym::Symbol)
#     println("Sherlock! Using catchall method")
#     return getfield.(obj, sym)
# end

# outputs = Outputs[Outputs{Float64,Float64}(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0), Outputs{Complex{Float64},Float64}(0.24285036855335476 - 0.0im, 0.09374476338448777 - 0.0im, 0.0, 0.0, 1.2308487178526557, 0.0, 0.2772045889709332, 0.3183884946733561, 4.534990314410568 - 0.0im, 1.5050759240380058, 0.13763120546656513, 1.4099532010335012, 0.5442681845675164, 0.9835773158795993, 0.9917546651665418), Outputs{Complex{Float64},Float64}(0.7217628407058008 - 0.0im, 0.21697762721530664 - 0.0im, 0.0, 0.0, 1.7326960734792525, 0.0, 0.2589657900820015, 0.245706809640596, 6.7673827545271354 - 0.0im, 1.6376114138887694, 0.05416136893272035, 1.569136116792526, 0.47171648663229143, 0.9996516983041511, 0.9998258339851752), Outputs{Complex{Float64},Float64}(1.2990599048174767 - 0.0im, 0.31355858702643263 - 0.0im, 0.0, 0.0, 2.0129587976546834, 0.0, 0.22688706695390065, 0.1431274468578776, 8.953184950774602 - 0.0im, 1.3892205399175588, 0.01383177005724867, 1.3505053059805476, 0.32597614162717065, 0.9989877735121309, 0.999493758615896), Outputs{Complex{Float64},Float64}(1.7619807902797593 - 0.0im, 0.3720701211173133 - 0.0im, 0.0, 0.0, 2.09694555206726, 0.0, 0.19036789138116791, 0.07972442835883256, 11.104190789025736 - 0.0im, 1.09077738226134, 0.019353098469995278, 1.067410153478332, 0.2254005419795043, 0.9960145868560558, 0.9980053040220056), Outputs{Complex{Float64},Float64}(2.2711845655764025 - 0.0im, 0.4405002858849476 - 0.0im, 0.0, 0.0, 2.173383411501406, 0.0, 0.1657314413795813, 0.0556670648944034, 13.265213925277973 - 0.0im, 0.9558687456440128, 0.02470688289088402, 0.9386953965963786, 0.18206164167668523, 0.9863118571582342, 0.9931323462450683), Outputs{Complex{Float64},Float64}(2.4372150640530514 - 0.0im, 0.4554901469388099 - 0.0im, 0.0, 0.0, 2.1629611788994447, 0.0, 0.15314916243164545, 0.04256438043828989, 14.343403366255124 - 0.0im, 0.8800488657666018, 0.02782677887567497, 0.8655033880045069, 0.16175358145153387, 0.9771513649187485, 0.9885096686015512), Outputs{Complex{Float64},Float64}(2.6371721454533454 - 0.0im, 0.47293752842306824 - 0.0im, 0.0, 0.0, 2.1681319604225773, 0.0, 0.14389224181868238, 0.0387234776796615, 15.424864001541529 - 0.0im, 0.8574901315613467, 0.028785248170534933, 0.8445005855017522, 0.15144859631086924, 0.9608451485828865, 0.9802270903126921), Outputs{Complex{Float64},Float64}(2.9035116834001253 - 0.0im, 0.5070499699969895 - 0.0im, 0.0, 0.0, 2.1978736826245666, 0.0, 0.13838725719185638, 0.03708290981589751, 16.51325595934029 - 0.0im, 0.8476745782368323, 0.029258614602765128, 0.8355344940565491, 0.14591218715080329, 0.9309406549944107, 0.9648526597332935), Outputs{Complex{Float64},Float64}(2.915378995839232 - 0.0im, 0.4988635277901568 - 0.0im, 0.0, 0.0, 2.132452708206054, 0.0, 0.12893626553987916, 0.029308601308119464, 17.59133845621036 - 0.0im, 0.7995946884337315, 0.03243082128604525, 0.7887875033803128, 0.13497295451969854, 0.888830318342529, 0.9427779793474862), Outputs{Complex{Float64},Float64}(2.672487776120149 - 0.0im, 0.4658323109725899 - 0.0im, 0.0, 0.0, 1.9807514705340548, 0.0, 0.11703583739096063, 0.014783645024415093, 18.662919158779463 - 0.0im, 0.7077670820565516, 0.039347704185111965, 0.6983307468035365, 0.121723672045743, 0.8261271529063575, 0.908915371696594), Outputs{Complex{Float64},Float64}(2.6753732822512957 - 0.0im, 0.4714559098077924 - 0.0im, 0.0, 0.0, 1.925896978460363, 0.0, 0.11765311844886973, 0.013955301300062325, 19.763948184544113 - 0.0im, 0.702698187395941, 0.039939768326604105, 0.6931521258065394, 0.1221476899224832, 0.689154508910361, 0.8301533044627125), Outputs{Complex{Float64},Float64}(2.397999681058518 - 0.0im, 0.4549131149780635 - 0.0im, 0.0, 0.0, 1.774720023483803, 0.0, 0.1305556999113481, 0.01385081781673042, 20.895071940836615 - 0.0im, 0.7020550192002749, 0.04000541332594471, 0.6908722123615864, 0.1310620817257423, 0.4256466033071096, 0.652415974135451), Outputs{Float64,Float64}(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)]
