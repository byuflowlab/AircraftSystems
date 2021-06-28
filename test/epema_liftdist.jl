include(joinpath(AS.topdirectory, "test", "EpemaData.jl"))

wing = EpemaData.wing
rotor = EpemaData.rotor
setup = EpemaData.setup
results = EpemaData.results

Vinf = setup[:"Vinf"]
vinfs = [Vinf]
plotstepi = [1]
alphas = [setup[:"wing_aoa"]]
wing_b = wing[:"span"]
wing_TR = wing["chord_tip"] / wing[:"chord_root"]
wing_AR = wing["span"]^2 / wing[:"area"]
wing_θroot = 0.0
wing_θtip = 0.0
rotor_omegas = [AS.get_omega(Vinf=setup[:"Vinf"], J=setup[:"J"], D=rotor[:"diameter"])]
nblades = [rotor[:"Nblades"]]
rhub = [rotor[:"radius_hub"] / 2]
rtip = [rotor[:"radius"]]
chord_data = rotor["r/R vs chord/R"]
twist_data = rotor["r/R vs twist"]

radii = [chord_data[:,1] * rtip[1]]
chords = [chord_data[:,2] * rtip[1]]
twists = [FM.linear(twist_data[:,1]*rtip[1], twist_data[:,2], radii[1])]
airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "e212-il.csv")
airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
airfoilname = "eppler212"
airfoilnames = [fill(airfoilname, length(radii[1]))]
index = [1]
rotor_positions = [[0.0; rotor[:"y"]; 0.0]]
rotor_orientations = [[-1.0; 0.0; 0.0]] # positive x downstream
spindirections = [true]

args = AS.vlm_bem_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip,
                        rotor_omegas, nblades, rhub, rtip, radii, chords, twists,
                        airfoilcontours, airfoilnames, index, rotor_positions, rotor_orientations,
                        spindirections,
                        Res_list = [fill([EpemaData.setup[:"Re"]], length(radii))],
                        surfacenames = ["epema wing"])

outs = AS.runsimulation!(args...)

aircraft = args[1]
parameters = args[2]

#! The values from Epema are normalized, I haven't normalized our results yet.
figure()
plot(aircraft.wingsystem.lifting_line_rs[1][2,2:end], parameters.cfs[1][3,:], label="BEM+VLM (not normalized yet)")
plot(EpemaData.results[:"lift distribution VLM"][:,1]*EpemaData.wing[:"span"]/2, EpemaData.results[:"lift distribution VLM"][:,2], label="Epema VLM model")
scatter(EpemaData.results[:"normalized cl experimental"][:,1]*EpemaData.wing[:"span"]/2, EpemaData.results[:"normalized cl experimental"][:,2], label="Epema experimental")
xlabel(L"y")
ylabel(L"c_l")
legend()

# no tests for now - eventually would like to add some tests on cl values, as well as add results to the plot
