include(joinpath(AS.topdirectory, "test", "EpemaData.jl"))

wing = EpemaData.wing
rotor = EpemaData.rotor
setup = EpemaData.setup
results = EpemaData.results

Vinf = setup[:"Vinf"]
Vref = Vinf
vinfs = [Vinf]
plotstepi = [1]
alphas = [setup[:"wing_aoa"]] .* pi/180
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
radii = radii_list
rotor_chords = chords_list
rotor_twists = twists_list
# radii = [rotor_chord_data[:,1] * rtip[1]]
# rotor_chords = [rotor_chord_data[:,2] * rtip[1]]
# rotor_twists = [FM.linear(rotor_twist_data[:,1]*rtip[1], rotor_twist_data[:,2], radii[1])] * pi/180

airfoilcontour = joinpath(contour_directory, "epema_interpolated_bspline_n30_0700.dat")
airfoilcontours = fill(airfoilcontour, length(radii))
# airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
index = [1]
rotor_positions = [[0.0; rotor[:"y"]; 0.0]]
rotor_orientations = [[-1.0; 0.0; 0.0]] # positive x downstream
spin_directions = [true]

epema_chords = EpemaData.wing[:"chords"]
xle = [[0.0, 0.0, 0.0]]
yle = [epema_chords[:,1]]
zle = [[0.0, 0.0, 0.0]]
wing_chord = [epema_chords[:,2]]
wing_twist = [[0.0, 0.0, 0.0]]
wing_phi = [[0.0, 0.0, 0.0]]
wing_le_sweep = 0.0
wing_ϕ = 0.0

surfacenames = ["epema wing"]

wing_npanels = 256
wing_n_chordwise_panels = 1
Res = [5e4, 1e5, 5e5, 1e6, 1e7]
Machs = [0.0, 0.1, 0.2, 0.3]

# Res_list = fill(fill(Res, length(radii[1])), length(nblades))
# Ms_list = fill(fill(Machs, length(radii[1])), length(nblades))
Res_list = fill(Res, length(nblades))
Ms_list = fill(Machs, length(nblades))

args = AS.vlm_bem_template(vinfs, plotstepi, alphas,
    xle, yle, zle, wing_chord, wing_twist, wing_phi, wing_npanels, wing_n_chordwise_panels,
    rotor_omegas, nblades, rhub, rtip, radii, rotor_chords, rotor_twists,
    airfoilcontours, index,
    rotor_positions, rotor_orientations, spin_directions, Res_list, Ms_list;
        Vref, symmetric = true, iref = 1, static_margin = 0.10, liftingline_x_over_c = 0.25,
        cambers = [fill((xc) -> 0, length(lex)) for lex in xle],
        spanwise_spacings = fill(VL.Uniform(), length(xle)),
        chordwise_spacings = fill(VL.Uniform(), length(xle)),
        wake_developement_factor = 1.0, # fully developed by default
        swirl_recovery_factor = 0.5, # as described in Veldhuis' paper
        surfacenames,
        polar_directory,
        rotor_names = ["rotor"],
        mass = 0.0,
        inertia_x = 0.0,
        inertia_y = 0.0,
        inertia_z = 0.0,
        plot_directory = joinpath(AS.topdirectory, "data","plots",TODAY),
        plot_base_name = "Epema_props_on",
        plot_extension = ".png",
        step_symbol = LS.L"\alpha ",
)

outs = AS.runsimulation!(args...)

aircraft = args[1]
parameters = args[2]
panels = aircraft.wing_system.system.surfaces[1]

span_plot = aircraft.wing_system.lifting_line_rs[1][2,2:end] / (wing[:"span"]/2) # normalized
cls_plot = parameters.cfs[1][1][3,:]
wing_chords = FM.linear(epema_chords[:,1], epema_chords[:,2], range(epema_chords[1,1], stop=epema_chords[3,1], length=length(span_plot)))
# cls_plot = cls_plot .* wing_chords / wing[:"mac"] #VortexLattice already normalizes it for now

gammas = [panel.gamma * Vref for panel in aircraft.wing_system.system.properties[1]]
velocities = [panel.velocity * Vref for panel in aircraft.wing_system.system.properties[1]]
velocities_minus_rotor = velocities .- parameters.wake_function[1].(VL.top_center.(panels)) #w/o rotor-induced velocities
# cr = CartesianIndices(panels)
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

epema_fig = plt.figure("epema_blown_wing_cl_epema_polar")
epema_fig.clear()
plt.plot(span_plot, cls_plot, label="VortexLattice output")
plt.plot(results[:"lift distribution VLM rotors on"][:,1], results[:"lift distribution VLM rotors on"][:,2], "--", label="Epema VLM model")
plt.scatter(results[:"normalized cl experimental"][:,1], results[:"normalized cl experimental"][:,2], label="Epema experimental")
plt.plot(span_plot, cls_new, "--", label="2 * Γ / V_{infty}")
plt.plot(span_plot, cls_new2, "--", label="2 * Γ (V_{infty} + V_w)/ V_{infty}^2")
plt.plot(span_plot, cls_new3, "--", label="2 * Γ (V_{infty} + V_w + V_r)/ V_{infty}^2")
plt.xlabel("2y/b")
plt.ylabel("normalized c_l (c/c_{mac})")
plt.legend()
if save_figs
    plt.savefig(joinpath(plot_directory,"epema_blown_wing_cl_epema_polar" * ".pdf"), bbox_inches="tight")
end

# no tests for now - eventually would like to add some tests on cl values, as well as add results to the plot
