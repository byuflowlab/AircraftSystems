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
wing_le_sweep = 0.0
wing_ϕ = 0.0
rotor_omegas = [AS.get_omega(Vinf=setup[:"Vinf"], J=setup[:"J"], D=rotor[:"diameter"])]
nblades = [rotor[:"Nblades"]]
rhub = [rotor[:"radius_hub"] / 2]
rtip = [rotor[:"radius"]]
chord_data = rotor["r/R vs chord/R"]
twist_data = rotor["r/R vs twist"]
q = 1/2 * 1.0 * Vinf^2

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
spin_directions = [true]
polardirectory=joinpath(AS.topdirectory, "data","airfoil","polars","20210618")

args = AS.lift_distribution_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ,
                        Res_list = [fill([EpemaData.setup[:"Re"]], length(radii))],
                        surfacenames = ["epema wing"], Vinf=Vinf)

outs = AS.runsimulation!(args...)

aircraft = args[1]
parameters = args[2]

span_plot = aircraft.wingsystem.lifting_line_rs[1][2,2:end] / (EpemaData.wing[:"span"]/2)
cls_plot = parameters.cfs[1][3,:]
wing_chords = FM.linear(epema_chords[:,1], epema_chords[:,2], range(epema_chords[1,1], stop=epema_chords[3,1], length=length(span_plot)))
cls_plot = cls_plot .* wing_chords / EpemaData.wing[:"mac"]

epema_chords = EpemaData.wing[:"chords"]
epema_cl = EpemaData.results[:"lift distribution VLM rotors off"][:,2]
epema_x = EpemaData.results[:"lift distribution VLM rotors off"][:,1]
epema_cl_interpolated = FM.linear(epema_x, epema_cl, span_plot)
epema_cl_interpolated = [[0 0 cl] for cl in epema_cl_interpolated]

gammas = [panel.gamma for panel in aircraft.wingsystem.system.properties[1]]
velocities = [panel.velocity for panel in aircraft.wingsystem.system.properties[1]]

cls_new = 2 * gammas' ./ wing_chords .* wing_chords / EpemaData.wing[:"mac"]

panels = aircraft.wingsystem.system.surfaces[1]
Δs = VL.top_vector.(panels)
Vfreestream = VL.freestream_velocity(aircraft.wingsystem.system.freestream[1])

# calculating gammas from epema data
# Vs = [Vfreestream + vel for vel in velocities]

# gammas_epema = zeros(length(wing_chords))
# for i in 1:length(wing_chords)
#     test_gamma = epema_cl_interpolated[i]' * Vinf^2 .* wing_chords[i] ./ (2 * cross(Vs[i], Δs[i]))[3]
#     gammas_epema[i] = test_gamma[3]
# end

figure()
plot(span_plot, cls_plot, label="BEM+VLM")
plot(EpemaData.results[:"lift distribution VLM rotors off"][:,1], epema_cl, label="Epema VLM")
plot(span_plot, cls_new, label=L"2 * Γ / V_{\infty}")
xlabel(L"2y/b")
ylabel(L"normalized\ c_l (c/c_{mac})")
legend()
