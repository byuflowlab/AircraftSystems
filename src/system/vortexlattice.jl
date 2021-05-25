#=##############################################################################################
Filename: vortexlattice.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file is meant to provide convenience functions for building VortexLattice objects.
        This includes convenience constructors for common wing configurations.
=###############################################################################################

function simplewing(;
    wing_b = 2.0, wing_TR = 0.8, wing_AR = 8.0, wing_θroot = 0.0, wing_θtip = 0.0,
    wing_camber = fill((xc) -> 0, 2), # camberline function for each section
    wing_npanels = 50, wing_nchordwisepanels = 1,
    wing_spacing_s = VL.Cosine(), wing_spacing_c = VL.Uniform(),
    symmetric = true
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
    # maybe find a better way to build this

    return vlmsystem
end

function VL.Freestream(freestream::Freestream)
    VL.Freestream(freestream.vinf, freestream.alpha, freestream.beta, freestream.Omega)
end
