
alphas = [0.0, 4.0, 10.0] .* pi/180
ploti = 1:length(alphas)
wing_b = 640e-3 * 2
wing_TR = 1.0 # hershey bar
wing_c = 240e-3
wing_AR = wing_b / wing_c
wing_θroot = 0.0
wing_θtip = 0.0
wing_le_sweep = 0.0
wing_ϕ = 0.0
plotbasename = "PROWIM_props_off"
data_PROWIM_lift_distribution = AS.lift_distribution_template(ploti, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ;
    plotdirectory = joinpath(AS.topdirectory, "data","plots",AS.TODAY),
    plotbasename = plotbasename,
    plotextension = ".pdf",
    stepsymbol = LS.L"\alpha ",
    surfacenames = ["PROWIM, props off"]
)
objective = AS.runsimulation!(data_PROWIM_lift_distribution...)

prowim_lift_distribution_props_off = PROWIMData.props_off["lift_distribution"]

fig = plt.figure(plotbasename * "_cf_distribution")
fig_lift = plt.figure(plotbasename * "_lift_distribution")
axs = fig.get_axes()
ax_lift = fig_lift.get_axes()[1]
alpha_labels = [0,4,10]
for (idata, data) in enumerate(prowim_lift_distribution_props_off)
    cratio = idata / length(alpha_labels)
    axs[3].scatter(data[:,1], data[:,2], marker = "x", color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=LS.L"PROWIM, \alpha = " * "$(alpha_labels[idata])" * LS.L"^\circ")
    ax_lift.scatter(data[:,1] ./ wing_b * 2, data[:,2], marker = "x", color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=LS.L"PROWIM, \alpha = " * "$(alpha_labels[idata])" * LS.L"^\circ")
end
axs[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))
ax_lift.legend(loc="upper left", bbox_to_anchor=(1.01,1))
fig.tight_layout()
fig.savefig(joinpath(plotdirectory,"PROWIM_props_off_lift_distribution.pdf"), bbox_inches="tight")
fig.tight_layout()
fig.savefig(joinpath(plotdirectory,"PROWIM_props_off_lift_distribution.pdf"), bbox_inches="tight")
# fig.savefig(joinpath(notebookdirectory,"PROWIM_propsoff_lift_distribution.pdf"), bbox_inches="tight")

# @test objective == 0
