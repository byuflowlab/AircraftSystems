# Epema propeller
nblades_list = [6, 6, 6]
D = EpemaData.rotor["diameter"]
rtip_list = ones(3) * D / 2
radii_list = fill(rs_desired .* rtip_list[1], 3)
rhub_list = fill(radii_list[1][1], 3)
nJs = 27
Js = ones(3,nJs) .* range(0.01, stop=2.0, length=nJs)'
# Vinf = 19 # m/s
# ns = Vinf ./ Js / 2 / rtip_list[1]
# omegas = ns .* 2 * pi / 60
omegas = ones(3,nJs) .* 5000 * 2 * pi / 60 # rad/s, arbitrary RPM
vinf = 10.0 # or set vinf

chords_raw = EpemaData.rotor["chords_raw"]
chords_list = fill(AS.FM.linear(chords_raw[:,1], chords_raw[:,2], radii_list[1]), 3)

twists_raw = EpemaData.rotor["twists_raw"]
twist = AS.FM.linear(twists_raw[:,1] .* rtip_list[1], twists_raw[:,2], radii_list[1]) .* pi/180 # radians
# set collective at 70\% radius
twists_07_desired = [25.0, 30.0, 35.0] .* pi/180
twists_07 = AS.FM.linear(radii_list[1], twist, 0.7 * rtip_list[1])
twists_list = Vector{Vector{typeof(twist[1])}}(undef, 3)
twist_extra = 3.0 * pi/180
for (i, twistval) in enumerate(twists_07_desired)
    twists_list[i] = twist .+ twistval .- twists_07 .+ twist_extra
end

# extract scanned contour data, smooth, and interpolate to rs_desired
locations = string.(Int.(round.(rs_desired .* 1000, digits=0)))
locations[1:end-1] = "0" .* locations[1:end-1]

##########################################################
# uncomment the next 2 lines to use all the distinct polars:
# contourfilenames = "epema_interpolated_bspline_n30_" .* locations
# repeattag = ""
# or uncomment these lines to try just repeating the r/R = 0.7 airfoil:
contourfilenames = fill("epema_interpolated_bspline_n30_" * locations[8],length(locations))
repeattag = "_repeat07af"
##########################################################
contour_paths_list = fill(joinpath(contour_directory, contourfilenames[1] * ".dat"),3)
# contour_paths_list = fill(joinpath.(contour_directory, contourfilenames .* ".dat"),3)
airfoilnames = fill(contourfilenames, 3)

plotstepi = 1:length(Js)

mach_correction = nothing
# mach_correction = CC.PrandtlGlauert()

Res = [5e4, 1e5, 5e5, 1e6, 1e7]
Machs = [0.0, 0.1, 0.2, 0.3]

Res_lists = fill(Res, 3)
Ms_lists = fill(Machs, 3)
# Res_lists = fill(fill(Res, length(radii_list[1])), 3)
# Ms_lists = fill(fill(Machs, length(radii_list[1])), 3)

index = [1,2,3]
positions = [[0.0, 0.0, -10.0], [0.0, 0.0, 0.0], [0.0, 0.0, 10.0]]
orientations = fill([-1.0, 0.0, 0.0],3)
spin_directions = fill(true,3)

plot_base_name = "epema"
# data_epema_rotor_sweep = AS.rotor_sweep_template(Js, omegas, nblades, rhub_list, rtip_list, radii, chords, twists, airfoilcontours, airfoilnames, index, positions, orientations, spin_directions, Res_list, Ms_list;
#     polar_directory = polar_directory,
#     closefigure = true,
#     useoldfiles = true,
#     rotor_names = [LS.L"\theta_{r/R=0.7} = 25^\circ", LS.L"\theta_{r/R=0.7} = 30^\circ", LS.L"\theta_{r/R=0.7} = 35^\circ"],
#     plot_directory = joinpath(AS.topdirectory, "data", "plots", AS.TODAY),
#     plot_base_name = plot_base_name,
#     plot_extension = ".png"
#     )
data_epema_rotor_sweep = AS.rotor_sweep_template(Js, omegas, nblades_list, rhub_list, rtip_list, mach_correction,
    radii_list, chords_list, twists_list,
    Res_lists, Ms_lists, contour_paths_list,
    index, positions, orientations, spin_directions;
        polar_directory = joinpath(AS.topdirectory, "test", "data", "airfoil", "polars"),
        rotor_names = [LS.L"\theta_{r/R=0.7} = 25^\circ", LS.L"\theta_{r/R=0.7} = 30^\circ", LS.L"\theta_{r/R=0.7} = 35^\circ"],
        plot_polars = true,
        plot_directory = joinpath(AS.topdirectory, "data", "plots", AS.TODAY),
        plot_base_name = plot_base_name,
        plot_extension = ".png",
        step_symbol = LS.L"J",
        close_figure = false,
        use_old_files = true,
        use_old_corrected_files = true
)

step_range = data_epema_rotor_sweep[end-1]
objective = AS.runsimulation!(data_epema_rotor_sweep...)

epema_eta = EpemaData.epema_eta

epema_ct = EpemaData.epema_ct

# Epema xrotor data
ct_xrotor = EpemaData.ct_xrotor

fig = plt.figure(plot_base_name * "_rotor_sweep")
axs = fig.get_axes()

# plot CTs
for (i,data) in enumerate(epema_ct)
    cratio = i / length(epema_ct)
    axs[1].scatter(data[:,1], data[:,2], marker = "o", s = 100, facecolors="none", edgecolors = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="Experiment")
    axs[1].scatter(ct_xrotor[i][:,1], marker = "x", ct_xrotor[i][:,2], color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="XROTOR")
end
axs[1].legend(loc="upper left", bbox_to_anchor=(1.01,1))

# plot efficiencies
for (i,data) in enumerate(epema_eta)
    cratio = i / length(epema_eta)
    axs[3].scatter(data[:,1], data[:,2], marker = "o", s = 100, facecolors="none", edgecolors = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="Experiment")
end

# save figure
savename = plot_base_name * "_rotor_sweep_extratwist_$(round(twist_extra * 180/pi; digits = 0))" * repeattag * ".pdf"
fig.savefig(joinpath(plot_directory, savename), bbox_inches="tight")
