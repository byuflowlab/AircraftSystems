"""
    post_plot_cf_cm_alpha_sweep(aircraft, parameters, alpharange, step_symbol) <: PostAction

# Arguments:

* `aircraft::Aircraft`: aircraft system struct
* `parameters <: Parameters`: inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `alpharange::AbstractArray`: range of alphas in radians
* `step_symbol::String`: defines the step, e.g. `alpha` or `time`

# Modifies:

* `aircraft::Aircraft`
* `parameters <: Parameters`

# Returns:

* `flag::Bool`: true if an action experiences errors
* saves plots in `parameters.plot_directory`

`parameters <: Parameters` requires:

* `CFs::Array{Float64,2}` : [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
* `CMs::Array{Float64,2}` : [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth step
* `plot_directory::String`: directory where plots are saved
* `plot_base_name::String`: first portion of the saved figure file name
* `plot_extension::String`: extension of saved figure files

"""
function post_plot_cf_cm_alpha_sweep(aircraft, parameters, alpharange, step_symbol)

    # extract info
    CFs = parameters.CFs
    CMs = parameters.CMs
    plot_directory = parameters.plot_directory
    plot_base_name = parameters.plot_base_name
    plot_extension = parameters.plot_extension

    # check data
    @assert isdir(plot_directory) "plot_directory $plot_directory does not exist"

    # prepare figures
    fig = plt.figure(plot_base_name * "_cl_alpha_sweep")
    fig.clear()
    fig.add_subplot(312, ylabel = L"C_D")
    fig.add_subplot(313, ylabel = L"C_Y", xlabel = step_symbol)
    fig.add_subplot(311, ylabel = L"C_L")
    axs = fig.get_axes()

    fig_moment = plt.figure(plot_base_name * "_cm_alpha_sweep")
    fig_moment.clear()
    fig_moment.add_subplot(311, ylabel = L"C_{M,x}")
    fig_moment.add_subplot(312, ylabel = L"C_{M,y}")
    fig_moment.add_subplot(313, ylabel = L"C_{M,z}", xlabel = step_symbol)
    axs_moment = fig_moment.get_axes()

    # plot
    for (i,ax) in enumerate(axs)
        ax.plot(alpharange .* 180/pi, CFs[i,:], label = "VortexLattice")
        axs_moment[i].plot(alpharange .* 180/pi, CMs[i,:], label = "VortexLattice")
    end

    # save
    savepath = joinpath(plot_directory, plot_base_name * "_cl_alpha_sweep" * plot_extension)
    fig.savefig(savepath, bbox_inches="tight")
    return false
end

"""
    post_plot_cf_cm_alpha_sweep(aircraft, step_range) <: Action

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft`: aircraft system to be simulated
* `step_range::AbstractArray`: defines each step of the simulation

# Returns:

* `CFs::Array{Float64,2}` : [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
* `CMs::Array{Float64,2}` : [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth step
* `plot_directory::String`: directory where plots are saved
* `plot_base_name::String`: first portion of the saved figure file name
* `plot_extension::String`: extension of saved figure files

"""
function post_plot_cf_cm_alpha_sweep(aircraft, step_range)

    CFs = zeros(3, length(step_range))
    CMs = zeros(3, length(step_range))
    plot_directory = ""
    plot_base_name = "default"
    plot_extension = ".pdf"

    return CFs, CMs, plot_directory, plot_base_name, plot_extension
end
