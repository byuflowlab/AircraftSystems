# benchmarking
# [extract_rotor_info, build_operating_points, select_rotor, solve_ccblade, nondimensionalize]
labels = ["(x10^5) extract rotor info", "(x10^3) build_operating_points", "(x10^4) select_rotor", "(x10^0) solve_ccblade", "(x10^3) nondimensionalize"]
kronos_epema_rotor_sweep = [8.53e-2, 4.2694e-2, 1.6446e-2, 0.034703221, 9.54845e-2]
kronos_epema_blown_wing = [9.96e-2, 4.9103e-2, 6.97e-3, 0.245689797, 7.3299e-2]
kronos_epema_blown_wing_simplified = [1.136e-1, 4.1276e-2, 1.371e-2, 0.068328487, 7.6982e-2]
# kronos_epema_rotor_sweep = [8.53e-7, 4.2694e-5, 1.644e-6, 0.034703221, 9.5484e-5]
# kronos_epema_blown_wing = [9.96e-7, 4.9103e-5, 6.9e-7, 0.245689797, 7.3299e-5]
# kronos_epema_blown_wing_simplified = [1.136e-6, 4.1276e-5, 1.371e-6, 0.068328487, 7.6982e-5]
kronos = [kronos_epema_rotor_sweep, kronos_epema_blown_wing, kronos_epema_blown_wing_simplified]
tick_labels = ["epema_rotor_sweep", "epema_blown_wing", "epema_blown_wing_simplified"]

benchmark_fig = plt.figure("benchmark_ccblade")
benchmark_fig.clear()
benchmark_ax = benchmark_fig.add_subplot(1,1,1)
towers = []
for (i,time) in enumerate(kronos)
    local xs = ones(length(time)) .* i
    local heights = time
    # heights = deepcopy(time)
    local widths = ones(length(time)) .* 0.4
    local ys = [vcat(0,heights)[i] for i = 1:length(heights)]
    for j in 1:length(xs)
        local cratio = (j-1)/(length(xs)-1)
        if j == 1
            benchmark_ax.bar(xs[j], heights[j], widths[j], ys[j], tick_label=tick_labels[i], label = i == 1 ? labels[j] : nothing, color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio))
        else
            benchmark_ax.bar(xs[j], heights[j], widths[j], ys[j], label = i == 1 ? labels[j] : nothing, color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio))
        end
    end
end

benchmark_ax.set_ylabel("time [s]")
benchmark_ax.set_xticks(1:length(kronos))
benchmark_ax.set_xticklabels(["epema rotor sweep", "epema blown wing", "epema simplified blown wing"])
# benchmark_ax.set_xticklabels(vcat([tick_labels[i] for i in 1:length(kronos) for j in 1:length(kronos_epema_blown_wing)]))
benchmark_ax.legend()
benchmark_fig.tight_layout()

benchmark_fig.savefig(joinpath(plot_directory, "benchmark_ccblade.png"), bbox_inches="tight")
