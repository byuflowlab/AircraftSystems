include(joinpath(AS.topdirectory, "test", "VeldhuisData.jl"))

wing = VeldhuisData.wing
rotor = VeldhuisData.rotor
setup = VeldhuisData.setup
results = VeldhuisData.results

Vinf = setup[:"Vinf"]
vinfs = [Vinf]
plotstepi = [1]
alphas = [setup[:"wing_aoa1"]]
wing_b = wing[:"span"]
wing_TR = wing["chord_tip"] / wing[:"chord_root"]
wing_AR = wing["span"]^2 / wing[:"area"]
wing_θroot = 0.0
wing_θtip = 0.0
rotor_omegas = [AS.get_omega(Vinf=setup[:"Vinf"], J=setup[:"J"], D=rotor[:"diameter"])]
nblades = [rotor[:"Nblades"]]
rhub = [rotor[:"radius_hub"] / 2]
rtip = [rotor[:"radius"]]
rotor_chord_data = rotor["r/R vs chord/R"]
rotor_twist_data = rotor["r/R vs twist"]

radii = [rotor_chord_data[:,1] * rtip[1]]
rotor_chords = [rotor_chord_data[:,2] * rtip[1]]
rotor_twists = [FM.linear(rotor_twist_data[:,1]*rtip[1], rotor_twist_data[:,2], radii[1])]
airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "e212-il.csv")
airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
airfoilname = "eppler212"
airfoilnames = [fill(airfoilname, length(radii[1]))]
index = [1]
rotor_positions = [[0.0; rotor[:"y"]; 0.0]]
rotor_orientations = [[-1.0; 0.0; 0.0]] # positive x downstream
spindirections = [true]

veldhuis_chords = VeldhuisData.wing[:"chords"]
xle = [0.0, 0.0]
yle = veldhuis_chords[:,1]
zle = [0.0, 0.0]
wing_chord = veldhuis_chords[:,2]
wing_twist = [0.0, 0.0]
wing_phi = [0.0, 0.0]

# Res_list = [fill([VeldhuisData.setup[:"Re"]], length(radii))]
surfacenames = ["epema wing"]
polardirectory=joinpath(AS.topdirectory, "data","airfoil","polars","20210618")

args = AS.vlm_bem_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip,
                        rotor_omegas, nblades, rhub, rtip, radii, rotor_chords, rotor_twists,
                        airfoilcontours, airfoilnames, index, rotor_positions, rotor_orientations,
                        spindirections;
                        # Res_list,
                        surfacenames,
                        polardirectory,
                        # other wing keyword arguments
                        xle, yle, zle, wing_chord, wing_twist, wing_phi, Vinf=Vinf, Vref=Vinf)

outs = AS.runsimulation!(args...)

aircraft = args[1]
parameters = args[2]

span_plot = aircraft.wingsystem.lifting_line_rs[1][2,2:end] / (wing[:"span"]/2) # normalized
cls_plot = parameters.cfs[1][3,:]
wing_chords = FM.linear(veldhuis_chords[:,1], veldhuis_chords[:,2], range(veldhuis_chords[1,1], stop=veldhuis_chords[2,1], length=length(span_plot)))
cls_plot = cls_plot .* wing_chords / wing[:"mac"]

gammas = [panel.gamma for panel in aircraft.wingsystem.system.properties[1]]
# velocities = [panel.velocity for panel in aircraft.wingsystem.system.properties[1]]

cls_new = 2 * gammas' ./ wing_chords .* wing_chords / wing[:"mac"]

figure()
plot(span_plot, cls_plot, label="BEM+VLM")
scatter(results[:"normalized cl distribution 0 deg aoa rotors on"][:,1], results[:"normalized cl distribution 0 deg aoa rotors on"][:,2], label="Veldhuis experimental")
plot(span_plot, cls_new, label=L"2 * Γ / V_{\infty}")
xlabel(L"2y/b")
ylabel(L"normalized\ c_l (c/c_{mac})")
legend()

# no tests for now - eventually would like to add some tests on cl values, as well as add results to the plot