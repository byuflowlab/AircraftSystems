"Blown wing system."
struct System{R}
    # CCBlade objects
    rotors::Vector{CC.Rotor}
    sections_list::Vector{Vector{CC.Section}}
    # VortexLattice objects
    VL_system::VL.System
    VL_reference::VL.Reference
    # coupling
    rotorindex::Vector{R}                   # which element of `System.rotors`
    rotorposition::Vector{Tuple{R,R,R}}     # position of each rotor
    rotororientation::Vector{Vector{R}}     # forward axial orientation of each rotor
    wakedevelopmentfactor::Vector{R}        # 0-1: set how developed each wake is
end

function System(
    rotors::Vector{CC.Rotor},
    sections_list::Vector{Vector{CC.Section}},
    VL_system::VL.System,
    VL_reference::VL.Reference,
    rotorindex::Vector,
    rotorposition::Vector{Tuple{R,R,R}} where R,
    rotororientation::Vector{Vector},
    wakedevelopmentfactor::Vector
)
    @assert length(rotors) == length(sections_list) "`rotors` and `sections_list` must be of the same length"
    @assert length(rotorindex) == length(rotorposition) "`rotorindex` and `rotorposition` must be of the same length"
    @assert length(rotororientation) == length(rotorposition) "`rotorposition` and `rotororientation` must be of the same length"
    @assert length(wakedevelopmentfactor) == length(rotorindex) "`rotorindex` and `wakedevelopmentfactor` must be of the same length"

    return System(rotors, sections_list, VL_system, VL_reference, rotorindex, rotorposition, rotororientation, wakedevelopmentfactor)
end

"Convenience constructor for a single wing, single rotor `System` object."
function SimpleSystem(;
    rotor_D, rotor_Rhub, rotor_r, rotor_c, rotor_θ,
    rotor_orientation = [-1.0, 0.0, 0.0],
    rotor_position = [0.0, 0.0, 0.0],
    wing_b = 2.0, wing_TR = 0.8, wing_AR = 8.0, wing_θroot = 0.0, wing_θtip = 0.0,
    wing_camber = fill((xc) -> 0, 2), # camberline function for each section
    wing_npanels = 50, wing_nchordwisepanels = 1,
    wing_spacing_s = VL.Cosine(), wing_spacing_c = VL.Uniform()
)
    # build rotor

    # build wing
    vlmsystem = simplewing(;
        wing_b = 2.0, wing_TR = 0.8, wing_AR = 8.0, wing_θroot = 0.0, wing_θtip = 0.0,
        wing_camber = fill((xc) -> 0, 2), # camberline function for each section
        wing_npanels = 50, wing_nchordwisepanels = 1,
        wing_spacing_s = VL.Cosine(), wing_spacing_c = VL.Uniform()
    )

end

function simplewing(;
    wing_b = 2.0, wing_TR = 0.8, wing_AR = 8.0, wing_θroot = 0.0, wing_θtip = 0.0,
    wing_camber = fill((xc) -> 0, 2), # camberline function for each section
    wing_npanels = 50, wing_nchordwisepanels = 1,
    wing_spacing_s = VL.Cosine(), wing_spacing_c = VL.Uniform()
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
    phi = [0.0, 0.0]

    # discretization parameters
    ns = wing_npanels
    nc = wing_nchordwisepanels

    # construct surfaces and grids
    grid, surface = VL.wing_to_surface_panels(xle, yle, zle, chord, theta, phi, ns, nc;
        fc = wing_camber, spacing_s=wing_spacing_s, spacing_c=wing_spacing_c)
    grids = [grid]
    surfaces = [surface]

    symmetric = true

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
    # maybe find a better way to build this

    return vlmsystem
end
