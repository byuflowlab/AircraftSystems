include(joinpath(AS.topdirectory, "test", "EpemaData.jl"))

wing = EpemaData.wing
rotor = EpemaData.rotor
setup = EpemaData.setup
results = EpemaData.results

Vinf = setup[:"Vinf"]
Vref = Vinf
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
rhub = [rotor[:"radius_hub"]]
rtip = [rotor[:"radius"]]
rotor_chord_data = rotor["r/R vs chord/R"]
rotor_twist_data = rotor["r/R vs twist"]
radii = [rotor_chord_data[:,1] * rtip[1]]
rotor_chords = [rotor_chord_data[:,2] * rtip[1]]
rotor_twists = [FM.linear(rotor_twist_data[:,1]*rtip[1], rotor_twist_data[:,2], radii[1])] * pi/180

airfoilcontour = joinpath(contourdirectory, "epema_interpolated_bspline_n30_0700.dat")
airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
airfoilname = "epema_interpolated_bspline_n30_0700"
airfoilnames = [fill(airfoilname, length(radii[1]))]
index = [1]
rotor_positions = [[0.0; rotor[:"y"]; 0.0]]
rotor_orientations = [[-1.0; 0.0; 0.0]] # positive x downstream
spindirections = [true]

epema_chords = EpemaData.wing[:"chords"]
xle = [0.0, 0.0, 0.0]
yle = epema_chords[:,1]
zle = [0.0, 0.0, 0.0]
wing_chord = epema_chords[:,2]
wing_twist = [0.0, 0.0, 0.0]
wing_phi = [0.0, 0.0, 0.0]
wing_le_sweep = 0.0
wing_ϕ = 0.0

surfacenames = ["epema wing"]

wing_npanels = 256
Res = [5e4, 1e5, 5e5, 1e6, 1e7]
Machs = [0.0, 0.1, 0.2, 0.3]

Res_list = fill(fill(Res, length(radii[1])), length(nblades))
Ms_list = fill(fill(Machs, length(radii[1])), length(nblades))
println("rotor_positions = $rotor_positions")
args = AS.vlm_bem_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ,
                        rotor_omegas, nblades, rhub, rtip, radii, rotor_chords, rotor_twists,
                        airfoilcontours, airfoilnames, index, rotor_positions, rotor_orientations,
                        spindirections, Res_list, Ms_list;
                        surfacenames,
                        polardirectory,
                        # other wing keyword arguments
                        xle, yle, zle, chord = wing_chord, twist = wing_twist, phi = wing_phi, Vinf, Vref, npanels = wing_npanels)

outs = AS.runsimulation!(args...)

aircraft = args[1]
parameters = args[2]
panels = aircraft.wingsystem.system.surfaces[1]

span_plot = aircraft.wingsystem.lifting_line_rs[1][2,2:end] / (wing[:"span"]/2) # normalized
cls_plot = parameters.cfs[1][3,:]
wing_chords = FM.linear(epema_chords[:,1], epema_chords[:,2], range(epema_chords[1,1], stop=epema_chords[3,1], length=length(span_plot)))
cls_plot = cls_plot .* wing_chords / wing[:"mac"]

gammas = [panel.gamma * Vref for panel in aircraft.wingsystem.system.properties[1]]
velocities = [panel.velocity * Vref for panel in aircraft.wingsystem.system.properties[1]]
velocities_minus_rotor = velocities .- parameters.wakefunctions[1].(VL.top_center.(panels)) #w/o rotor-induced velocities
cr = CartesianIndices(panels)
Δs = VL.top_vector.(panels)

# back of the envelope approach
cls_new = 2 * gammas' ./  (Vinf * wing[:"mac"])

# including wing induced velocities
v_perp = LA.cross.(velocities_minus_rotor, Δs) ./ LA.norm.(Δs)
v_perp = LA.norm.([[v[1]; 0.0; v[3]] for v in v_perp])
cls_new2 = 2 * gammas' .* v_perp' ./ (wing[:"mac"] * Vinf^2)

# including wing and rotor induced velocities
v_perp3 = LA.cross.(velocities, Δs) ./ LA.norm.(Δs)
v_perp3 = LA.norm.([[v[1]; 0.0; v[3]] for v in v_perp3])
cls_new3 = 2 * gammas' .* v_perp3' ./ (wing[:"mac"] * Vinf^2)

plt.figure()
plt.plot(span_plot, cls_plot, label="BEM+VLM")
plt.plot(results[:"lift distribution VLM rotors on"][:,1], results[:"lift distribution VLM rotors on"][:,2], label="Epema VLM model")
plt.scatter(results[:"normalized cl experimental"][:,1], results[:"normalized cl experimental"][:,2], label="Epema experimental")
plt.plot(span_plot, cls_new, label="2 * Γ / V_{infty}")
plt.plot(span_plot, cls_new2, label="2 * Γ (V_{infty} + V_w)/ V_{infty}^2")
plt.plot(span_plot, cls_new3, label="2 * Γ (V_{infty} + V_w + V_r)/ V_{infty}^2")
plt.xlabel("2y/b")
plt.ylabel("normalized c_l (c/c_{mac})")
plt.legend()

# no tests for now - eventually would like to add some tests on cl values, as well as add results to the plot
