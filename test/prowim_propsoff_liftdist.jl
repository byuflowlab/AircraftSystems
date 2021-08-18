alphas = PROWIMData.props_off["lift_distribution_alpha"]
ploti = 1:length(alphas)
wing_b = PROWIMData.wing["span"]
wing_TR = PROWIMData.wing["TR"] # 1.0 # hershey bar
wing_c = PROWIMData.wing["chord"] # 240e-3
wing_AR = wing_b / wing_c
wing_θroot = PROWIMData.wing["theta_root"]
wing_θtip = PROWIMData.wing["theta_tip"]
wing_le_sweep = PROWIMData.wing["le_sweep"]
wing_ϕ = PROWIMData.wing["dihedral"]

plot_base_name = "PROWIM_props_off"
data_PROWIM_lift_distribution = AS.lift_distribution_template(ploti, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ;
    plot_directory = joinpath(AS.topdirectory, "data","plots",AS.TODAY),
    plot_base_name = plot_base_name,
    plot_extension = ".pdf",
    step_symbol = LS.L"\alpha ",
    surfacenames = ["PROWIM, props off"]
)
objective = AS.runsimulation!(data_PROWIM_lift_distribution...)

prowim_lift_distribution_props_off = PROWIMData.props_off["lift_distribution"]

fig = plt.figure(plot_base_name * "_cf_distribution")
fig_lift = plt.figure(plot_base_name * "_lift_distribution")
axs = fig.get_axes()
ax_lift = fig_lift.get_axes()[1]
alpha_labels = Int.(round.(alphas .* 180/pi, digits=0))
for (idata, data) in enumerate(prowim_lift_distribution_props_off)
    cratio = idata / length(alpha_labels)
    axs[3].scatter(data[:,1], data[:,2], marker = "x", color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=LS.L"PROWIM, \alpha = " * "$(alpha_labels[idata])" * LS.L"^\circ")
    ax_lift.scatter(data[:,1] ./ wing_b * 2, data[:,2], marker = "x", color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=LS.L"PROWIM, \alpha = " * "$(alpha_labels[idata])" * LS.L"^\circ")
end
axs[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))
ax_lift.legend(loc="upper left", bbox_to_anchor=(1.01,1))
fig.tight_layout()
fig.savefig(joinpath(plot_directory,"PROWIM_props_off_lift_distribution.pdf"), bbox_inches="tight")
fig.tight_layout()
fig.savefig(joinpath(plot_directory,"PROWIM_props_off_lift_distribution.pdf"), bbox_inches="tight")
# fig.savefig(joinpath(notebookdirectory,"PROWIM_propsoff_lift_distribution.pdf"), bbox_inches="tight")

# @test objective == 0
