import AircraftSystems
AS = AircraftSystems
FM = AS.FM
using LaTeXStrings
plt = AS.plt
VL = AS.VL
using Test

# topdirectory = normpath(joinpath(@__DIR__,".."))

# @testset "rotor.jl" begin

# test rotor_sweep_template
# Js = range(2.0, stop=4, length=17)
# omegas = fill(50.0, length(Js))
# nblades = 3
# radii = [0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-2/2
# rhub = radii[1]
# rtip = radii[end]
# chords = [9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] * 1e-3
# twists = [35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0] * pi/180
# airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "naca4412.dat")
# airfoilcontours = fill(airfoilcontour, length(radii))
# airfoilname = "naca4412"
# airfoilnames = fill(airfoilname, length(radii))
# polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", "20210524")

# simulationdata = AS.rotor_sweep_template(Js, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, Res_list = [fill([5e4, 1e5, 1e6], length(radii))]; polardirectory = polardirectory)
# objective = AS.runsimulation!(simulationdata...)

# parameters = simulationdata[2]
# plt.plot(parameters.Js,)

# test wing_clalpha_template

# wingsystem = simplewing()
# aircraft = Aircraft()

# reference = VL.Reference()



Vinf = 10.0
alpha = 5.0 * pi/180
beta = 0.0
Omegas = zeros(3)
# freestream = VL.Freestream(Vinf, alpha, beta, Omegas)
# interpret freestream
vlmfreestream = VL.Freestream(Vinf, alpha, beta, Omegas)

lexs = [[0.0,0.0], [2.0,2.0]]
leys = [[0.0,1.0], [0.0,1.0]]
lezs = [[0.0,0.0], [0.0,0.0]]
chords = [[0.2,0.2], [0.2,0.2]]
thetas = [[0.0,0.0], [0.0,0.0]]
phis = [[0.0,0.0], [0.0,0.0]]
nspanwisepanels = [30,30]
nchordwisepanels = [1,1]
wingsystem = AS.VortexLatticeSystem(lexs, leys, lezs, chords, thetas, phis, nspanwisepanels, nchordwisepanels)
vlmsystem = wingsystem.system
# solve vortex lattice
VL.steady_analysis!(vlmsystem, vlmsystem.surfaces, vlmsystem.reference[1], vlmfreestream; symmetric=true)#, additional_velocity = vwake)
# extract forces and moments
CF, CM = VL.body_forces(vlmsystem; frame=VL.Wind())
# get lifting line locations
# rs = VL.lifting_line_r.(wingsystem.grids, Ref(0.25))
# chords = VL.lifting_line_c.(wingsystem.grids)

cfs, cms = VL.lifting_line_coefficients(wingsystem.system, wingsystem.lifting_line_rs, wingsystem.lifting_line_chords)

fig = plt.figure("liftdistribution")
axs = []
push!(axs, fig.add_subplot(312))
push!(axs, fig.add_subplot(313))
push!(axs, fig.add_subplot(311))

labels = ["wing1", "wing2"]

for (i,cf) in enumerate(cfs)
    rs_plot = AS.get_midpoints(wingsystem.lifting_line_rs[i][2,:])
    for (j,ax) in enumerate(axs)
        ax.plot(rs_plot, cf[j,:], label=labels[i])
    end
end

axs[1].set_ylim([-5,5])
axs[1].set_ylabel(L"c_d")
# axs[1].legend()
axs[2].set_ylim([-1,1])
axs[2].set_ylabel(L"c_y")
axs[2].set_xlabel(L"y [m]")
axs[2].legend()
axs[3].set_ylim([0,50])
axs[3].set_ylabel(L"c_l")
# axs[3].legend()

# # geometry (right half of the wing)
# xle = [0.0, 0.4]
# yle = [0.0, 7.5]
# zle = [0.0, 0.0]
# chord = [2.2, 1.8]
# theta = [2.0*pi/180, 2.0*pi/180]
# phi = [0.0, 0.0]
# fc = fill((xc) -> 0, 2) # camberline function for each section

# # discretization parameters
# ns = 12
# nc = 6
# spacing_s = VL.Uniform()
# spacing_c = VL.Uniform()

# # reference parameters
# Sref = 30.0
# cref = 2.0
# bref = 15.0
# rref = [0.50, 0.0, 0.0]
# Vinf = 1.0
# ref = VL.Reference(Sref, cref, bref, rref, Vinf)

# # freestream parameters
# alpha = 1.0*pi/180
# beta = 0.0
# Omega = [0.0; 0.0; 0.0]
# fs = VL.Freestream(Vinf, alpha, beta, Omega)

# # construct surface
# grid, surface = VL.wing_to_surface_panels(xle, yle, zle, chord, theta, phi, ns, nc;
#     fc = fc, spacing_s=spacing_s, spacing_c=spacing_c)

# x_over_c = 0.25
# r = VL.lifting_line_r(grid, x_over_c)
# c = VL.lifting_line_c(grid)

# # create vector containing all surfaces
# surfaces = [surface]

# # we can use symmetry since the geometry and flow conditions are symmetric about the X-Z axis
# symmetric = true

# # perform steady state analysis
# system = VL.steady_analysis(surfaces, ref, fs; symmetric=symmetric)

# # retrieve near-field forces
# CF, CM = VL.body_forces(system; frame=VL.Wind())

# # perform far-field analysis
# CDiff = VL.far_field_drag(system)

# CD, CY, CL = CF
# Cl, Cm, Cn = CM

# # r =
# # cf, cm = VL.lifting_line_coefficients(system, r, c)
