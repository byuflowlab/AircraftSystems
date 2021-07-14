# validate PROWIM cl-alpha sweep, props off:
alphas = range(-5, stop=12, length = 18) .* pi/180
wing_b = 640e-3 * 2
wing_TR = 1.0 # hershey bar
wing_c = 240e-3
wing_AR = wing_b / wing_c
wing_θroot = 0.0
wing_θtip = 0.0
plotbasename = "PROWIM_propsoff_CF"
simulationdata_CL_alpha = AS.cl_alpha_sweep_template(alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip;
        plotdirectory = joinpath(AS.topdirectory, "data","plots",TODAY),
        plotbasename = plotbasename,
        plotextension = ".pdf"
    )
objective = AS.runsimulation!(simulationdata_CL_alpha...)
alpha_prowim = [
    0;
    1.079545454545455;
    2.121212121212121;
    3.1818181818181825;
    4.242424242424243;
    5.303030303030304;
    6.344696969696968;
    7.386363636363636;
    8.446969696969695;
    9.469696969696969;
]
CLs_prowim = [
    0;
    0.07431693989071053;
    0.14644808743169413;
    0.2163934426229509;
    0.2863387978142078;
    0.35409836065573785;
    0.41530054644808756;
    0.47650273224043715;
    0.5420765027322405;
    0.6032786885245902
]

CDdata_balance = [
    0.0      0.0106943;
    1.89474  0.0106321;
    3.81579  0.0123731;
    5.73684  0.0174093;
    7.65789  0.0221969;
    9.55263  0.0295959;
]

CDdata_wakesurvey = [
    0.0      0.00988601;
    3.81579  0.0116269;
    9.57895  0.0277927;
]

fig_propsoff_clalpha = plt.figure(plotbasename * "_clalphasweep")
axs = fig_propsoff_clalpha.get_axes()
axs[1].scatter(alpha_prowim, CLs_prowim, marker = "x", label="Veldhuis")
axs[1].legend(loc="upper left", bbox_to_anchor=(1.01,1))
axs[2].scatter(CDdata_balance[:,1], CDdata_balance[:,2], marker = "+", label = "balance")
axs[2].scatter(CDdata_wakesurvey[:,1], CDdata_wakesurvey[:,2], marker = "x", label = "wake survey")
axs[2].legend(loc="upper left", bbox_to_anchor=(1.01,1))

fig_propsoff_clalpha.tight_layout()
fig_propsoff_clalpha.savefig(joinpath(plotdirectory,"PROWIM_propsoff_clalphasweep.pdf"), bbox_inches="tight")
# fig_propsoff_clalpha.savefig(joinpath(notebookdirectory,"PROWIM_propsoff_clalphasweep.pdf"), bbox_inches="tight")

# @test objective == 0
