# simulation controls
alphas = PROWIMData.props_on["lift_distribution_alpha"]
ploti = 1:length(alphas)
# wing definition
wing_b = PROWIMData.wing["span"]
wing_TR = PROWIMData.wing["TR"] # 1.0 # hershey bar
wing_c = PROWIMData.wing["chord"] # 240e-3
wing_AR = wing_b / wing_c
wing_θroot = PROWIMData.wing["theta_root"]
wing_θtip = PROWIMData.wing["theta_tip"]
wing_le_sweep = PROWIMData.wing["le_sweep"]
wing_ϕ = PROWIMData.wing["dihedral"]

wing_lexs = [[0.0, 0.0]]
wing_leys = [[0.0, wing_b/2]]
wing_lezs = [[0.0, 0.0]]
wing_chords = [[wing_c, wing_c]]
wing_thetas = [[wing_θroot, wing_θtip]]
wing_phis = [[wing_ϕ, wing_ϕ]]
wing_nspanwisepanels = 50
wing_nchordwisepanels = 1

# environment
environment = AS.Environment()
ν = environment.ν
# freestream definition
Re_c = PROWIMData.props_on["Re_c"]
vinf = Re_c * ν / wing_c
vinfs = fill(vinf, length(alphas))
# rotor definition

# Epema propeller w/ 4 blades to match
nblades = [PROWIMData.rotor["blades"]]
rtip = [PROWIMData.rotor["diameter"]/2]
radii = fill(PROWIMData.rotor["radii"], length(nblades))
rhub = fill(radii[1][1], length(nblades))
J = PROWIMData.rotor["J"]
D = PROWIMData.rotor["diameter"]
n = vinf / J / D # rev/s
omega = n .* 2 * pi # rad/s
omegas = ones(length(nblades),length(alphas)) .* omega # rad/s
chords_raw = PROWIMData.rotor["chords_raw"]
chords = fill(AS.FM.linear(chords_raw[:,1], chords_raw[:,2], radii[1]), length(nblades))

twists_raw = PROWIMData.rotor["twists_raw"]
twist = AS.FM.linear(twists_raw[:,1] .* rtip[1], twists_raw[:,2], radii[1]) .* pi/180 # radians
# set collective to 30 deg. at 70\% radius
twists_07_desired = [PROWIMData.rotor["twist_07"]]
twists_07 = AS.FM.linear(radii[1], twist, 0.7 * rtip[1])
twists = Vector{Vector{typeof(twist[1])}}(undef, length(nblades))
twist_extra = -0.3 * pi/180
for (i, twistval) in enumerate(twists_07_desired)
    twists[i] = twist .+ twistval .- twists_07 .+ twist_extra
end

locations = string.(Int.(round.(rs_desired .* 1000, digits=0)))
locations[1:end-1] = "0" .* locations[1:end-1]
# uncomment below to use distinct airfoils at each section
# contourfilenames = "epema_interpolated_bspline_n30_" .* locations
# uncomment below to repeat the r/R = 0.7 airfoil across the blade
contour_filenames = fill("epema_interpolated_bspline_n30_" * locations[8],length(radii[1]))
contour_paths = fill(joinpath.(contour_directory, contour_filenames .* ".dat"),length(nblades))

polar_directory = polar_directory
plotstepi = 1:length(alphas)

Res = [5e4, 1e5, 5e5, 1e6, 1e7]
Machs = [0.0, 0.1, 0.2, 0.3]

Res_list = fill(fill(Res, length(radii[1])), length(nblades))
Ms_list = fill(fill(Machs, length(radii[1])), length(nblades))

index = [1]
rotor_positions = [[-PROWIMData.rotor["x"], PROWIMData.rotor["y"], 0.0]]
rotor_orientations = [[-1.0, 0.0, 0.0]]
spin_directions = [true]

# call template
plot_base_name = "PROWIM_props_on"
wake_developement_factors = [1.0] #[0.0, 0.5, 1.0]
swirl_recovery_factors = [0.5] #[0.0, 0.5, 1.0]
wake_developement_factor = 1.0 #[0.0, 0.5, 1.0]
swirl_recovery_factor = 0.5 #[0.0, 0.5, 1.0]

data_PROWIM_vlm_bem = AS.vlm_bem_template(vinfs, plotstepi, alphas,
    wing_lexs, wing_leys, wing_lezs, wing_chords, wing_thetas, wing_phis, wing_nspanwisepanels, wing_nchordwisepanels,
    omegas, nblades, rhub, rtip, radii, chords, twists,
    contour_paths, index,
    rotor_positions, rotor_orientations, spin_directions, Res_list, Ms_list;
        Vref = 1.0, symmetric = true, iref = 1, static_margin = 0.10, liftingline_x_over_c = 0.25,
        cambers = [fill((xc) -> 0, length(lex)) for lex in wing_lexs],
        spanwise_spacings = fill(VL.Uniform(), length(wing_lexs)),
        chordwise_spacings = fill(VL.Uniform(), length(wing_lexs)),
        polar_directory,
        wake_developement_factor = 1.0, # fully developed by default
        swirl_recovery_factor = 0.5, # as described in Veldhuis' paper
        surfacenames = ["wing"],
        rotor_names = ["rotor"],
        mass = 0.0,
        inertia_x = 0.0,
        inertia_y = 0.0,
        inertia_z = 0.0,
        plot_directory = joinpath(AS.topdirectory, "data","plots",TODAY),
        plot_base_name,
        plot_extension = ".png",
        step_symbol = LS.L"\alpha "
)
objective = AS.runsimulation!(data_PROWIM_vlm_bem...)

# plot validation
waketag = "_wdf_$(round(wake_developement_factor; digits=1))_srf_$(round(swirl_recovery_factor; digits=1))"
fig_rotor = plt.figure(plot_base_name * "_rotor_sweep")
ax_rotor = fig_rotor.get_axes()[1]
ax_rotor.scatter(0.85, 0.168, c="r", label="Velduis")
ax_rotor.legend(loc="upper left", bbox_to_anchor=(1.01,1))
fig_rotor.tight_layout()
fig_rotor.savefig(joinpath(plot_directory,"PROWIM_props_on_rotor_sweep" * waketag * ".pdf"), bbox_inches="tight")
# fig_rotor.savefig(joinpath(notebookdirectory,"PROWIM_propson_rotorsweep.pdf"), bbox_inches="tight")

fig_cf = plt.figure(plot_base_name * "_cf_distribution")
# local fig_lift = plt.figure(plot_base_name * "_lift_distribution")
fig_lift = plt.figure(plot_base_name * "_lift_distribution")
axs_cf = fig_cf.get_axes()
# local ax_lift = fig_lift.get_axes()[1]
ax_lift = fig_lift.get_axes()[1]
cl_data_props_on = PROWIMData.props_on["lift_distribution"]
plot_labels = "Velduis, " .* [LS.L"\alpha = 0^\circ", LS.L"\alpha = 4^\circ", LS.L"\alpha = 10^\circ"]
# local aircraft = data_PROWIM_vlm_bem[1]
aircraft = data_PROWIM_vlm_bem[1]
b = aircraft.wing_system.lifting_line_rs[1][2,end]
for (i, data) in enumerate(cl_data_props_on)
    cratio = i / length(cl_data_props_on)
    axs_cf[3].scatter(data[:,1] .* b, data[:,2], marker="+", color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=plot_labels[i])
    ax_lift.scatter(data[:,1], data[:,2], marker="+", color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=plot_labels[i])
end
axs_cf[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))
fig_cf.tight_layout()
fig_cf.savefig(joinpath(plot_directory,"PROWIM_props_on_cf_distribution" * waketag * ".pdf"), bbox_inches="tight")
ax_lift.legend(loc="upper left", bbox_to_anchor=(1.01,1))
fig_lift.tight_layout()
fig_lift.savefig(joinpath(plot_directory,"PROWIM_props_on_lift_distribution" * waketag * ".pdf"), bbox_inches="tight")
# fig_cf.savefig(joinpath(notebookdirectory,"PROWIM_propson_lift_distribution" * waketag * ".pdf"), bbox_inches="tight")

fig_CF = plt.figure(plot_base_name * "_cl_alpha_sweep")
axs_CF = fig_CF.get_axes()

# Fetch data.
# local CL_data = PROWIMData.props_on["CL_alpha"]
CL_data = PROWIMData.props_on["CL_alpha"]
# local CD_data_balance = PROWIMData.props_on["CD_balance"]
CD_data_balance = PROWIMData.props_on["CD_balance"]
# local CD_data_wakesurvey = PROWIMData.props_on["CD_wake_survey"]
CD_data_wakesurvey = PROWIMData.props_on["CD_wake_survey"]

axs_CF[3].scatter(CL_data[:,1], CL_data[:,2], marker = "+", label="Velduis")
axs_CF[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))
axs_CF[1].scatter(CD_data_balance[:,1], CD_data_balance[:,2], marker = "+", label="Velduis, balance")
axs_CF[1].scatter(CD_data_wakesurvey[:,1], CD_data_wakesurvey[:,2], marker = "+", label="Velduis, wake survey")
axs_CF[1].legend(loc="upper left", bbox_to_anchor=(1.01,1))
fig_CF.tight_layout()
fig_CF.savefig(joinpath(plot_directory,"PROWIM_props_on_cl_alpha_sweep" * waketag * ".pdf"), bbox_inches="tight")
# fig_CF.savefig(joinpath(notebookdirectory,"PROWIM_propson_clalphasweep" * waketag * ".pdf"), bbox_inches="tight")
