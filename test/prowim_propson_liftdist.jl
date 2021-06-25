# simulation controls
alphas = [0.0, 4.0, 10.0] .* pi/180
ploti = 1:length(alphas)
# wing definition
wing_b = 640e-3 * 2
wing_TR = 1.0 # hershey bar
wing_c = 240e-3
wing_AR = wing_b / wing_c
wing_θroot = 0.0
wing_θtip = 0.0
# environment
environment = AS.Environment()
ν = environment.ν
# freestream definition
Re_c = 0.8e6
vinf = Re_c * ν / wing_c
vinfs = fill(vinf, length(alphas))
# rotor definition
omegas = ones(1,length(alphas)) .* 1000
nblades = [4]
radii = [[0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-3/2]
rhub = [radii[1][1]]
rtip = [radii[1][end]]
chords = [[9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] * 1e-3]
twists = [[35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0]] # * pi/180 provide in degrees to match airfoil files

airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "naca4412.dat")
airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
airfoilname = "naca4412"
airfoilnames = [fill(airfoilname, length(radii[1]))]
index = [1]
rotor_X = [[-201.8e-3, 300e-3, 0.0]]
rotor_orientation = [[-1.0, 0.0, 0.0]]
spindirections = [true]

polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", "20210524")
plotstepi = 1:length(alphas)

simulationdata = AS.vlm_bem_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, rotor_X, rotor_orientation, spindirections, Res_list = [fill([5e4, 1e5, 1e6], length(radii))],
        polardirectory = polardirectory
    )
objective = AS.runsimulation!(simulationdata...)
