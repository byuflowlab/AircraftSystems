
alphas = [0.0, 4.0, 10.0] .* pi/180
ploti = 1:length(alphas)
wing_b = 640e-3 * 2
wing_TR = 1.0 # hershey bar
wing_c = 240e-3
wing_AR = wing_b / wing_c
wing_θroot = 0.0
wing_θtip = 0.0
plotbasename = "PROWIM_propsoff_cf"
simulationdata = AS.lift_distribution_template(ploti, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip;
    plotdirectory,
    plotbasename = plotbasename,
    plotextension = ".pdf",
    stepsymbol = L"\alpha ",
    surfacenames = ["PROWIM, w/o props"]
)

objective = AS.runsimulation!(simulationdata...)


y2b_cn_0deg_noprops = [
    0.09694654312188172 0.00459016393442635;
    0.17520171062009982 0.00721311475409836;
    0.2525958660014256 0.005901639344262133;
    0.31520456165359956 0.005901639344262355;
    0.37782466143977206 0.0006557377049181135;
    0.4091290092658591 0.0006557377049181135;
    0.5308624376336422 0.003278688524590123;
    0.5621667854597292 0.003278688524590123;
    0.6247811831789025 0.0006557377049181135;
    0.6865174625801855 0.0019672131147541183;
    0.8108652886671419 0.0019672131147541183;
    0.8717405559515325 -0.0006557377049178914;
    0.9186970776906629 -0.0006557377049181135
]

y2b_cn_4deg_noprops = [
    0.09710049893086248 0.3337704918032788;
    0.17448610121168928 0.3363934426229509;
    0.25274982181040634 0.3350819672131151;
    0.3153670705630795 0.33114754098360666;
    0.37714041339985754 0.31540983606557405;
    0.40933143264433364 0.30754098360655724;
    0.5311190306486102 0.28524590163934427;
    0.5615424091233072 0.2904918032786885;
    0.6250292230933714 0.2865573770491804;
    0.6867883107626516 0.277377049180328;
    0.7485759087669281 0.25508196721311494;
    0.8112330719885961 0.23278688524590174;
    0.8730434782608698 0.20000000000000018;
    0.9192045616535995 0.16590163934426239;
]

y2b_cn_10deg_noprops = [
    0.09708624376336422 0.740327868852459;
    0.1744803991446902 0.739016393442623;
    0.2518859586600142 0.7324590163934428;
    0.314520313613685 0.7206557377049181;
    0.3771746258018532 0.6996721311475411;
    0.4085217391304348 0.6800000000000002;
    0.5303292943692088 0.6485245901639345;
    0.5616136849607984 0.657704918032787;
    0.62424233784747 0.6485245901639345;
    0.686041339985745 0.6209836065573772;
    0.8114240912330721 0.5449180327868852;
    0.8724533143264435 0.47147540983606573;
    0.919552387740556 0.4059016393442625;
]

prowim_liftdistribution = [
    y2b_cn_0deg_noprops,
    y2b_cn_4deg_noprops,
    y2b_cn_10deg_noprops
]

fig = plt.figure(plotbasename * "_liftdistribution")
axs = fig.get_axes()
alpha_labels = [0,4,10]
for (idata, data) in enumerate(prowim_liftdistribution)
    cratio = idata / length(alpha_labels)
    axs[3].scatter(data[:,1], data[:,2], marker = "x", color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=L"PROWIM, \alpha = " * "$(alpha_labels[idata])" * L"^\circ")
end
axs[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))
fig.tight_layout()

if savefigs
    fig.savefig(joinpath(plotdirectory,"PROWIM_propsoff_liftdistribution.pdf"), bbox_inches="tight")
    fig.savefig(joinpath(notebookdirectory,"PROWIM_propsoff_liftdistribution.pdf"), bbox_inches="tight")
end

# @test objective == 0
