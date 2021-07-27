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
# environment
environment = AS.Environment()
ν = environment.ν
# freestream definition
Re_c = PROWIMData.props_on["Re_c"]
vinf = Re_c * ν / wing_c
vinfs = fill(vinf, length(alphas))
# rotor definition
# omegas = ones(1,length(alphas)) .* 1000
# nblades = [4]
# radii = [[0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-3/2]
# rhub = [radii[1][1]]
# rtip = [radii[1][end]]
# chords = [[9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] * 1e-3]
# twists = [[35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0]] # * pi/180 provide in degrees to match airfoil files

# airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "naca4412.dat")
# airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
# airfoilname = "naca4412"
# airfoilnames = [fill(airfoilname, length(radii[1]))]
# index = [1]
# rotor_X = [[-201.8e-3, 300e-3, 0.0]]
# rotor_orientation = [[-1.0, 0.0, 0.0]]
# spindirections = [true]

# polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", "20210524")
# plotstepi = 1:length(alphas)

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
# contourfilenames = "epema_interpolated_bspline_n30_" .* locations
# try just repeating the r/R = 0.7 airfoil and see how this looks
contourfilenames = fill("epema_interpolated_bspline_n30_" * locations[8],length(radii[1]))
airfoilcontours = fill(joinpath.(contourdirectory, contourfilenames .* ".dat"),length(nblades))
# airfoilcontours = fill(joinpath.(AS.topdirectory, "data", "airfoil", "contours", "20210702", contourfilenames .* ".dat"),length(nblades))
airfoilnames = fill(contourfilenames, length(nblades))

polardirectory = polardirectory
plotstepi = 1:length(alphas)

Res = [5e4, 1e5, 5e5, 1e6, 1e7]
Machs = [0.0, 0.1, 0.2, 0.3]

Res_list = fill(fill(Res, length(radii[1])), length(nblades))
Ms_list = fill(fill(Machs, length(radii[1])), length(nblades))

index = [1]
rotor_X = [[-PROWIMData.rotor["x"], PROWIMData.rotor["y"], 0.0]]
rotor_orientation = [[-1.0, 0.0, 0.0]]
spindirections = [true]

# call template
plotbasename = "PROWIM_props_on"
wakedevelopementfactors = [1.0] #[0.0, 0.5, 1.0]
swirlrecoveryfactors = [0.5] #[0.0, 0.5, 1.0]

for wakedevelopementfactor in wakedevelopementfactors
    for swirlrecoveryfactor in swirlrecoveryfactors
        local data_PROWIM_vlm_bem = AS.vlm_bem_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, rotor_X, rotor_orientation, spindirections, Res_list, Ms_list;
                wakedevelopementfactor = wakedevelopementfactor, # fully developed by default
                swirlrecoveryfactor = swirlrecoveryfactor, # as described in Veldhuis' paper
                polardirectory = polardirectory,
                surfacenames = ["wing"],
                rotornames = ["rotor"],
                plotbasename = plotbasename,
                plotextension = ".pdf"
            )
        local objective = AS.runsimulation!(data_PROWIM_vlm_bem...)

        # plot validation
        waketag = "_wdf_$(round(wakedevelopementfactor; digits=1))_srf_$(round(swirlrecoveryfactor; digits=1))"
        fig_rotor = plt.figure(plotbasename * "_rotor_sweep")
        ax_rotor = fig_rotor.get_axes()[1]
        ax_rotor.scatter(0.85, 0.168, c="r", label="Velduis")
        ax_rotor.legend(loc="upper left", bbox_to_anchor=(1.01,1))
        fig_rotor.tight_layout()
        fig_rotor.savefig(joinpath(plotdirectory,"PROWIM_props_on_rotor_sweep" * waketag * ".pdf"), bbox_inches="tight")
        # fig_rotor.savefig(joinpath(notebookdirectory,"PROWIM_propson_rotorsweep.pdf"), bbox_inches="tight")

        fig_cf = plt.figure(plotbasename * "_cf_distribution")
        local fig_lift = plt.figure(plotbasename * "_lift_distribution")
        axs_cf = fig_cf.get_axes()
        local ax_lift = fig_lift.get_axes()[1]
        cl_data_props_on = PROWIMData.props_on["lift_distribution"]
        plot_labels = "Velduis, " .* [LS.L"\alpha = 0^\circ", LS.L"\alpha = 4^\circ", LS.L"\alpha = 10^\circ"]
        local aircraft = data_PROWIM_vlm_bem[1]
        b = aircraft.wingsystem.lifting_line_rs[1][2,end]
        for (i, data) in enumerate(cl_data_props_on)
            cratio = i / length(cl_data_props_on)
            axs_cf[3].scatter(data[:,1] .* b, data[:,2], marker="+", color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=plot_labels[i])
            ax_lift.scatter(data[:,1], data[:,2], marker="+", color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=plot_labels[i])
        end
        axs_cf[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))
        fig_cf.tight_layout()
        fig_cf.savefig(joinpath(plotdirectory,"PROWIM_props_on_cf_distribution" * waketag * ".pdf"), bbox_inches="tight")
        ax_lift.legend(loc="upper left", bbox_to_anchor=(1.01,1))
        fig_lift.tight_layout()
        fig_lift.savefig(joinpath(plotdirectory,"PROWIM_props_on_lift_distribution" * waketag * ".pdf"), bbox_inches="tight")
        # fig_cf.savefig(joinpath(notebookdirectory,"PROWIM_propson_lift_distribution" * waketag * ".pdf"), bbox_inches="tight")

        fig_CF = plt.figure(plotbasename * "_cl_alpha_sweep")
        axs_CF = fig_CF.get_axes()

        # Fetch data.
        local CL_data = PROWIMData.props_on["CL_alpha"]
        local CD_data_balance = PROWIMData.props_on["CD_balance"]
        local CD_data_wakesurvey = PROWIMData.props_on["CD_wake_survey"]

        axs_CF[1].scatter(CL_data[:,1], CL_data[:,2], marker = "+", label="Velduis")
        axs_CF[1].legend(loc="upper left", bbox_to_anchor=(1.01,1))
        axs_CF[2].scatter(CD_data_balance[:,1], CD_data_balance[:,2], marker = "+", label="Velduis, balance")
        axs_CF[2].scatter(CD_data_wakesurvey[:,1], CD_data_wakesurvey[:,2], marker = "+", label="Velduis, wake survey")
        axs_CF[2].legend(loc="upper left", bbox_to_anchor=(1.01,1))
        fig_CF.tight_layout()
        fig_CF.savefig(joinpath(plotdirectory,"PROWIM_props_on_cl_alpha_sweep" * waketag * ".pdf"), bbox_inches="tight")
        # fig_CF.savefig(joinpath(notebookdirectory,"PROWIM_propson_clalphasweep" * waketag * ".pdf"), bbox_inches="tight")
    end
end
