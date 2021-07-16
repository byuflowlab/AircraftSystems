# validate PROWIM cl-alpha sweep, props off:
alphas = range(-5, stop=12, length = 18) .* pi/180
wing_b = PROWIMData.wing["span"]
wing_TR = PROWIMData.wing["TR"] # hershey bar
wing_c = PROWIMData.wing["chord"]
wing_AR = wing_b / wing_c
wing_θroot = PROWIMData.wing["theta_root"]
wing_θtip = PROWIMData.wing["theta_tip"]
wing_le_sweep = PROWIMData.wing["le_sweep"]
wing_ϕ = PROWIMData.wing["dihedral"]
plotbasename = "PROWIM_props_off"
data_PROWIM_CL_alpha = AS.cl_alpha_sweep_template(alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ;
        plotdirectory = joinpath(AS.topdirectory, "data","plots",AS.TODAY),
        plotbasename = plotbasename,
        plotextension = ".pdf"
    )
objective = AS.runsimulation!(data_PROWIM_CL_alpha...)

# Fetch data.
CL_data = PROWIMData.props_off["CL_alpha"]
CD_data_balance = PROWIMData.props_off["CD_alpha_balance"]
CD_data_wake_survey = PROWIMData.props_off["CD_alpha_wake_survey"]

# Update plots.
fig_props_off_cl_alpha = plt.figure(plotbasename * "_cl_alpha_sweep")
axs = fig_props_off_cl_alpha.get_axes()
axs[1].scatter(CL_data[:,1], CL_data[:,2], marker = "x", label="Veldhuis")
axs[1].legend(loc="upper left", bbox_to_anchor=(1.01,1))
axs[2].scatter(CD_data_balance[:,1], CD_data_balance[:,2], marker = "+", label = "balance")
axs[2].scatter(CD_data_wake_survey[:,1], CD_data_wake_survey[:,2], marker = "x", label = "wake survey")
axs[2].legend(loc="upper left", bbox_to_anchor=(1.01,1))

fig_props_off_cl_alpha.tight_layout()
if savefigs
    fig_props_off_cl_alpha.savefig(joinpath(plotdirectory,"PROWIM_props_off_cl_alpha_sweep.pdf"), bbox_inches="tight")
end

# @test objective == 0
