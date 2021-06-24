#=##############################################################################################
Filename: vortexlattice.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file is meant to provide convenience functions for building VortexLattice objects.
        This includes convenience constructors for common wing configurations.
=###############################################################################################
"""
Defines a wing system for use with VortexLattice.jl.

Contains:

* `system::VortexLattice.System` : a VortexLattice System struct
* `grids::Array{Array{Float64,2}}` : a vector of 2-D arrays, each 2-D array containing wing panel corner locations; returned by `VortexLattice.wing_to_surface_panels()`

"""
struct VortexLatticeSystem{T1, T2, T3, T4}
    system::T1
    lifting_line_rs::T2
    lifting_line_chords::T3
    airfoil_polars::T4
end

"""
Constructor for `VortexLatticeSystem`.

Inputs:

* `lexs::Array{Array{Float64,1}}` : array of vectors of x coordinates of the leading edge
* `leys::Array{Array{Float64,1}}` : array of vectors of y coordinates of the leading edge
* `lezs::Array{Array{Float64,1}}` : array of vectors of z coordinates of the leading edge
* `chords::Array{Array{Float64,1}}` : array of vectors of chord lengths
* `thetas::Array{Array{Float64,1}}` : array of vectors of twist angles in radians
* `phis::Array{Array{Float64,1}}` : array of vectors of dihedral angles in radians
* `nspanwisepanels::Array{Float64,1}` : vector of the number of desired spanwise panels
* `nchordwisepanels::Array{Float64,1}` : vector of the number of desired chordwise panels

Optional Inputs:

* `wing_cambers = [fill((xc) -> 0, length(lex)) for lex in lexs]` : vector of vectors of functions describing the camber line
* `spanwisespacings = fill(VL.Uniform(), length(lexs))` : vector of `<: VortexLattice.AbstractSpacing` objects describing the spanwise spacing
* `chordwisespacings = fill(VL.Uniform(), length(lexs))` : vector of `<: VortexLattice.AbstractSpacing` objects describing the chordwise spacing
* `Vref = 1.0` : velocity used to normalize
* `symmetric = true` : whether or not to mirror the geometry about the x-z plane
* `iref = 1` : index of `lexs`, `leys`, etc. to use when calculating reference properties
* `staticmargin = 0.10` : static margin of the `iref`th lifting surface

Returns:

* `vlmsystem::VortexLatticeSystem`

"""
function VortexLatticeSystem(lexs::AbstractArray{T}, leys, lezs, chords, thetas, phis, nspanwisepanels, nchordwisepanels;
        wing_cambers = [fill((xc) -> 0, length(lex)) for lex in lexs],
        spanwisespacings = fill(VL.Uniform(), length(lexs)),
        chordwisespacings = fill(VL.Uniform(), length(lexs)),
        Vref = 1.0, symmetric = true, iref = 1, staticmargin = 0.10, liftingline_x_over_c = 0.25
    ) where T <: AbstractArray

    @assert size(lexs) == size(leys) && size(lexs[1]) == size(leys[1])   "size of leading edge xs and ys not consistent"
    @assert size(lexs) == size(lezs) && size(lexs[1]) == size(lezs[1])   "size of leading edge xs and zs not consistent"
    @assert size(lexs) == size(chords) && size(lexs[1]) == size(chords[1]) "size of leading edge xs and chords not consistent"
    @assert size(lexs) == size(thetas) && size(lexs[1]) == size(thetas[1]) "size of leading edge xs and thetas not consistent"
    @assert size(lexs) == size(phis) && size(lexs[1]) == size(phis[1]) "size of leading edge xs and phis not consistent"

    nwings = length(lexs)
    # build grids and surfaces
    gridssurfaces = [VL.wing_to_surface_panels(lexs[i], leys[i], lezs[i], chords[i], thetas[i], phis[i], nspanwisepanels[i], nchordwisepanels[i];
        fc = wing_cambers[i], spacing_s=spanwisespacings[i], spacing_c=chordwisespacings[i]) for i in 1:nwings]
    # extract grids and surfaces
    grids = [gridsurface[1] for gridsurface in gridssurfaces]
    surfaces = [gridsurface[2] for gridsurface in gridssurfaces]
    # build reference
    ref = get_reference(lexs[iref], leys[iref], chords[iref], staticmargin, Vref)
    # build freestream
    alpha = 3.0 * pi/180
    beta = 0.0
    Omegas = zeros(3)
    vlmfreestream = VL.Freestream(Vref, alpha, beta, Omegas)
    # get lifting line geometry
    rs, chords = VL.lifting_line_geometry(grids)
    # build system
    system = VL.steady_analysis(surfaces, ref, vlmfreestream; symmetric=symmetric)
    vlmsystem = VortexLatticeSystem(system, rs, chords, nothing)

    return vlmsystem
end

"""
Convencience constructor for a `VortexLatticeSystem` with a single lifting surface. I.e., each argument has one less dimension:

Inputs:

* `lexs::Array{Float64,1}` : vector of x coordinates of the leading edge
* `leys::Array{Float64,1}` : vector of y coordinates of the leading edge
* `lezs::Array{Float64,1}` : vector of z coordinates of the leading edge
* `chords::Array{Float64,1}` : vector of chord lengths
* `thetas::Array{Float64,1}` : vector of twist angles in radians
* `phis::Array{Float64,1}` : vector of dihedral angles in radians
* `nspanwisepanels::Int64` : number of desired spanwise panels
* `nchordwisepanels::Int64` : number of desired chordwise panels

Optional Inputs:

* `wing_cambers = [fill((xc) -> 0, length(lex)) for lex in lexs]` : vector of vectors of functions describing the camber line
* `spanwisespacings = fill(VL.Uniform(), length(lexs))` : vector of `<: VortexLattice.AbstractSpacing` objects describing the spanwise spacing
* `chordwisespacings = fill(VL.Uniform(), length(lexs))` : vector of `<: VortexLattice.AbstractSpacing` objects describing the chordwise spacing
* `Vref = 1.0` : velocity used to normalize
* `symmetric = true` : whether or not to mirror the geometry about the x-z plane
* `iref = 1` : index of `lexs`, `leys`, etc. to use when calculating reference properties
* `staticmargin = 0.10` : static margin of the `iref`th lifting surface

Returns:

* `vlmsystem::VortexLatticeSystem`

"""
function VortexLatticeSystem(lexs, leys, lezs, chords, thetas, phis, nspanwisepanels, nchordwisepanels)
    @assert !(typeof(lexs[1]) <: AbstractArray) "error in dispatch for VortexLatticeSystem"
    return VortexLatticeSystem([lexs], [leys], [lezs], [chords], [thetas], [phis], [nspanwisepanels], [nchordwisepanels];
            kwargs...
        )
end

function get_reference(lex, ley, chord, staticmargin, Vref)
    bref = ley[end] * 2
    Sref = FM.trapz(ley, chord) # area of the starboard wing
    cref = Sref / bref * 2
    lex_midpoint = get_midpoints(lex)
    dy = [ley[i+1] - ley[i] for i in 1:length(ley)-1]
    xmean = weightedmean(lex_midpoint, dy) # might be worth testing
    rref = [xmean + cref * (0.25 - staticmargin), 0.0, 0.0]
    Vinf = Vref
    ref = VL.Reference(Sref, cref, bref, rref, Vinf)

    return ref
end

weightedmean(set, weights) = sum(set .* weights) / sum(weights)

get_midpoints(a) = [(a[i] + a[i+1])/2 for i in 1:length(a)-1]

"""
    simplewingsystem(; wing_b=2.0, wing_TR=0.8, wing_AR=8.0, wing_θroot=0.0, wing_θtip=0.0,
        xle=[0.0, 0.0], yle=[0.0, wing_b/2], zle=[0.0, 0.0], 
        cmac=wing_b / wing_AR, # AR = b/c => c = b/AR
        Sref=wing_b * cmac,
        chord=[cmac * 2 / (1 + wing_TR), cmac * 2 / (1/wing_TR + 1)], twist=[wing_θroot, wing_θtip], phi=[0.0,0.0],
        staticmargin=0.10,
        Vinf=1.0, #freestream velocity
        Vref=Vinf, # in case you want the reference velocity to be different than the freestream velocity
        alpha=0.0, # angle of attack in radians
        beta=0.0, # sideslip angle in radians
        Omega=[0.0, 0.0, 0.0], # rotational velocity around the reference location
        wing_camber=fill((xc) -> 0, 2), # camberline function for each section
        wing_npanels=50, wing_nchordwisepanels=1,
        symmetric=true, # if true, we don't need to mirror geometry. Good option if parameters/geometry is symmetric and you don't care about lateral stability derivatives
        spacing_s = symmetric ? VL.Cosine() : VL.Sine(), # if mirrored, sine spacing becomes cosine spacing
        spacing_c=VL.Uniform(), # should only ever need uniform chordwise spacing
        mirror=!symmetric,
        kwargs...)

Convenience constructor for a single wing `VortexLatticeSystem`.

Keyword Arguments:

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
* `staticmargin`: static margin of the wing
* `Vinf`: freestream velocity
* `Vref`: reference velocity; defaults to freestream velocity
* `alpha`: angle of attack (radians)
* `beta`: sideslip angle (radians)
* `Omega`: rotational velocity around the reference location
* `wing_camber`: camberline function for each section
* `wing_npanels`: number of spanwise panels for the wing
* `wing_nchordwisepanels`: number of chordwise panels for the wing
* `symmetric::Bool`: Flag for each surface indicating whether a mirror image across the X-Z plane should be used when calculating induced velocities. Defaults to true. From my understanding, this solves the half span and uses a mirrored aproach for induced velocities, whereas mirror will physically mirror the geometry for you and solve it all together (Tyler).
* `mirror::Bool`: mirrors the geometry over the x-z plane
* `wing_spacing_s::Int`: spanwise panel spacing scheme for the wing; defaults to Cosine (or Sine if mirror==true, which results in Cosine anyway)
* `wing_spacing_c::Int`: chordwise panel spacing scheme for the wing; defaults to Uniform()

"""
function simplewingsystem(; wing_b=2.0, wing_TR=0.8, wing_AR=8.0, wing_θroot=0.0, wing_θtip=0.0,
            xle=[0.0, 0.0], yle=[0.0, wing_b/2], zle=[0.0, 0.0], 
            cmac=wing_b/wing_AR, # AR = b/c => c = b/AR
            Sref=wing_b*cmac,
            wing_chord=[cmac*2/(1+wing_TR), cmac*2/(1/wing_TR+1)], wing_twist=[wing_θroot, wing_θtip], wing_phi=[0.0,0.0],
            staticmargin=0.10,
            Vinf=1.0, 
            Vref=Vinf, # in case you want the reference velocity to be different than the freestream velocity
            alpha=0.0,
            beta=0.0,
            Omega=[0.0, 0.0, 0.0], 
            wing_camber=fill((xc) -> 0, length(xle)),
            wing_npanels=50, wing_nchordwisepanels=1,
            symmetric=true, # if true, we don't need to mirror geometry. Good option if parameters/geometry is symmetric and you don't care about lateral stability derivatives
            mirror=!symmetric,
            spacing_s = symmetric ? VL.Cosine() : VL.Sine(), # if mirrored, sine spacing becomes cosine spacing
            spacing_c=VL.Uniform(), # should only ever need uniform chordwise spacing
            kwargs...)

    #=
    cr + ct = cmac * 2
    ct = λ * cr
    cr = cmac * 2 / (1 + λ)
    ct = cmac * 2 / (1/λ + 1)
    =#
    
    # Check for errors
    @assert size(xle) == size(yle)    "size of leading edge xs and ys not consistent"
    @assert size(yle) == size(zle)    "size of leading edge ys and zs not consistent"
    @assert size(xle) == size(wing_chord)    "size of leading edge xs and wing_chord not consistent"
    @assert size(xle) == size(wing_twist)    "size of leading edge xs and wing_twist not consistent"
    @assert size(xle) == size(wing_phi)    "size of leading edge xs and wing_phi not consistent"
    @assert yle[1] == 0.0    "yle must begin at the root (0.0)"
    @assert yle[end] == wing_b/2    "yle must end at the tip (span/2)"
    @assert !(symmetric && mirror)    "symmetric and mirror cannot both be true"
  
    # construct surfaces and grids
    grid, surface = VL.wing_to_surface_panels(xle, yle, zle, wing_chord, wing_twist, wing_phi, wing_npanels, wing_nchordwisepanels;
                        fc = wing_camber, spacing_s, spacing_c, mirror)

    grids = [grid]
    surfaces = [surface]

    # reference parameters
    cgx = cmac * (0.25 - staticmargin)
    rref = [cgx, 0.0, 0.0]
    reference = VL.Reference(Sref, cmac, wing_b, rref, Vref)

    # initialize freestream parameters
    fs = VL.Freestream(Vinf, alpha, beta, Omega)

    # perform steady state analysis
    vlmsystem = VL.steady_analysis(surfaces, reference, fs; symmetric)
    lifting_line_rs, lifting_line_chords = VL.lifting_line_geometry(grids)
    wingsystem = VortexLatticeSystem(vlmsystem, lifting_line_rs, lifting_line_chords, nothing)

    return wingsystem
end


"""
Convenience constructor for a simple airplane `VortexLatticeSystem`.

Inputs:

* `wing_b = 2.0` : wing span
* `wing_TR = 0.8` : wing taper ratio
* `wing_AR = 8.0` : wing aspect ratio
* `wing_θroot = 0.0` : wing root twist
* `wing_θtip = 0.0` : wing tip twist
* `wing_phi = 3 * pi/180.0` : wing dihedral angle
* `tail_b = 0.4` : tail span
* `tail_TR = 0.7` : tail taper ratio
* `tail_AR = 6.0` : tail aspect ratio
* `tail_θroot = 0.0` : tail root twist
* `tail_θtip = 0.0` : tail tip twist
* `tail_incidence = 3.0 * pi/180` : tail incidence angle (subtracts from twist)
* `tail_phi = 0.0` : tail dihedral angle
* `tail_position = [1.5, 0.0, 0.1]` : position of the y=0 leading edge point of the tail
* `vertical_tail_b = 0.4` : span of the vertical tail
* `vertical_tail_TR = 0.7` : taper ratio of the vertical tail
* `vertical_tail_AR = 6.0` : aspect ratio of the vertical tail

Optional Inputs:

* `wing_camber = fill((xc) -> 0, 2)` : camberline function for each section
* `wing_npanels = 50` : number of spanwise panels for the wing
* `wing_nchordwisepanels = 3` : number of chordwise panels for the wing
* `wing_spacing_s = VL.Cosine()` : spanwise panel spacing scheme for the wing
* `wing_spacing_c = VL.Uniform()` : chordwise panel spacing scheme for the wing
* `tail_camber = fill((xc) -> 0, 2)` : camberline function for each section
* `tail_npanels = 20` : number of spanwise panels for the tail
* `tail_nchordwisepanels = 1` : number of chordwise panels for the tail
* `tail_spacing_s = VL.Cosine()` : spanwise panel spacing scheme for the tail
* `tail_spacing_c = VL.Uniform()` : chordwise panel spacing scheme for the tail
* `vertical_tail_camber = fill((xc) -> 0, 2)` : camberline function for each section
* `vertical_tail_npanels = 20` : number of spanwise panels for the vertical tail
* `vertical_tail_nchordwisepanels = 1` : number of chordwise panels for the vertical tail
* `vertical_tail_spacing_s = VL.Cosine()` : spanwise panel spacing scheme for the vertical tail
* `vertical_tail_spacing_c = VL.Uniform()` : chordwise panel spacing scheme for the vertical tail
* `symmetric = [true, true, false]` : mirrors the geometry over the x-z plane

"""
function simpleairplanesystem(wing_b = 2.0, wing_TR = 0.8, wing_AR = 8.0, wing_θroot = 0.0, wing_θtip = 0.0, wing_phi = 3 * pi/180.0, tail_b = 0.4, tail_TR = 0.7, tail_AR = 6.0, tail_θroot = 0.0, tail_θtip = 0.0, tail_incidence = 3.0 * pi/180, tail_phi = 0.0, tail_position = [1.5, 0.0, 0.1], vertical_tail_b = 0.4, vertical_tail_TR = 0.7, vertical_tail_AR = 6.0;
    wing_camber = fill((xc) -> 0, 2), # camberline function for each section
    wing_npanels = 50, wing_nchordwisepanels = 3,
    wing_spacing_s = VL.Cosine(), wing_spacing_c = VL.Uniform(),
    tail_camber = fill((xc) -> 0, 2), # camberline function for each section
    tail_npanels = 20, tail_nchordwisepanels = 1,
    tail_spacing_s = VL.Cosine(), tail_spacing_c = VL.Uniform(),
    vertical_tail_camber = fill((xc) -> 0, 2), # camberline function for each section
    vertical_tail_npanels = 20, vertical_tail_nchordwisepanels = 1,
    vertical_tail_spacing_s = VL.Cosine(), vertical_tail_spacing_c = VL.Uniform(),
    symmetric = [true, true, false], kwargs...
)
    # build wing object
    xle = [0.0, 0.0]
    yle = [0.0, wing_b/2]
    zle = [0.0, 0.0]
    cmac = wing_b / wing_AR # AR = b/c => c = b/AR
    #=
    cr + ct = cmac * 2
    ct = λ * cr
    cr = cmac * 2 / (1 + λ)
    ct = cmac * 2 / (1/λ + 1)
    =#
    chord = [cmac * 2 / (1 + wing_TR), cmac * 2 / (1/wing_TR + 1)]
    theta = [wing_θroot, wing_θtip]
    phi = [wing_phi, wing_phi]

    # discretization parameters
    ns = wing_npanels
    nc = wing_nchordwisepanels

    # construct surfaces and grids
    wing_grid, wing_surface = VL.wing_to_surface_panels(xle, yle, zle, chord, theta, phi, ns, nc;
        fc = wing_camber, spacing_s=wing_spacing_s, spacing_c=wing_spacing_c)

    # build tail object
    xle_tail = [tail_position[1], tail_position[1]]
    yle_tail = [0.0, tail_b/2]
    zle_tail = [tail_position[3], tail_position[3]]
    cmac_tail = tail_b / tail_AR # AR = b/c => c = b/AR
    #=
    cr + ct = cmac * 2
    ct = λ * cr
    cr = cmac * 2 / (1 + λ)
    ct = cmac * 2 / (1/λ + 1)
    =#
    chord_tail = [cmac_tail * 2 / (1 + tail_TR), cmac_tail * 2 / (1/tail_TR + 1)]
    theta_tail = [tail_θroot, tail_θtip] .- tail_incidence
    phi_tail = [tail_phi, tail_phi]

    # discretization parameters
    ns_tail = tail_npanels
    nc_tail = tail_nchordwisepanels

    # construct surfaces and grids
    tail_grid, tail_surface = VL.wing_to_surface_panels(xle_tail, yle_tail, zle_tail, chord_tail, theta_tail, phi_tail, ns_tail, nc_tail;
    fc = tail_camber, spacing_s=tail_spacing_s, spacing_c=tail_spacing_c)

    # build vertical tail
    xle_vertical_tail = [tail_position[1], tail_position[1]]
    yle_vertical_tail = [0.0, 0.0]
    zle_vertical_tail = [tail_position[3], tail_position[3] + vertical_tail_b]
    cmac_vertical_tail = vertical_tail_b / vertical_tail_AR # AR = b/c => c = b/AR
    #=
    cr + ct = cmac * 2
    ct = λ * cr
    cr = cmac * 2 / (1 + λ)
    ct = cmac * 2 / (1/λ + 1)
    =#
    chord_vertical_tail = [cmac_vertical_tail * 2 / (1 + vertical_tail_TR), cmac_vertical_tail * 2 / (1/vertical_tail_TR + 1)]
    theta_vertical_tail = [0.0, 0.0]
    phi_vertical_tail = [0.0, 0.0]

    # discretization parameters
    ns_vertical_tail = vertical_tail_npanels
    nc_vertical_tail = vertical_tail_nchordwisepanels

    # construct surfaces and grids
    vertical_tail_grid, vertical_tail_surface = VL.wing_to_surface_panels(xle_vertical_tail, yle_vertical_tail, zle_vertical_tail, chord_vertical_tail, theta_vertical_tail, phi_vertical_tail, ns_vertical_tail, nc_vertical_tail;
    fc = vertical_tail_camber, spacing_s=vertical_tail_spacing_s, spacing_c=vertical_tail_spacing_c)

    # assemble system
    grids = [wing_grid, tail_grid, vertical_tail_grid]
    surfaces = [wing_surface, tail_surface, vertical_tail_surface]

    symmetric = symmetric

    # reference parameters
    Sref = wing_b * cmac
    cref = cmac
    bref = wing_b
    staticmargin = 0.10
    cgx = cmac * (0.25 - staticmargin)
    rref = [cgx, 0.0, 0.0]
    Vinf = 1.0
    reference = VL.Reference(Sref, cref, bref, rref, Vinf)

    # initialize freestream parameters
    Vinf = 1.0
    alpha = 0.0*pi/180
    beta = 0.0
    Omega = [0.0; 0.0; 0.0]
    fs = VL.Freestream(Vinf, alpha, beta, Omega)

    # perform steady state analysis
    vlmsystem = VL.steady_analysis(surfaces, reference, fs; symmetric=symmetric)
    wingsystem = VortexLatticeSystem(vlmsystem, grids)

    return wingsystem
end

function VL.Freestream(freestream::Freestream)
    VL.Freestream(freestream.vinf, freestream.alpha, freestream.beta, freestream.Omega)
end
