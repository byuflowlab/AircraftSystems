"""
    post_plot_rotor_sweep(aircraft, parameters, step_range, step_symbol)

# Arguments:

* `aircraft::Aircraft`: aircraft system struct
* `parameters <: Parameters`: inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `step_range::AbstractArray`: range of times defining simulation
* `step_symbol::String`: defines the step, e.g. `alpha` or `time`

# Modifies:

* `aircraft::Aircraft`
* `parameters <: Parameters`

# Returns:

* `flag::Bool`: true if an action experiences errors

`parameters <: Parameters` requires:

* `Js::AbstractArray`: array of advance ratios of the ith rotor at the jth step
* `CTs::Array{Float64,2}`: array of thrust coefficients of the ith rotor at the jth step
* `CQs::Array{Float64,2}`: array of torque coefficients of the ith rotor at the jth step
* `ηs::Array{Float64,2}`: array of propulsive efficiencies of the ith rotor at the jth step
* `rotor_names::Vector{String}`: vector of rotor names for use in plot legend
* `plot_directory::String`: directory where plots are saved
* `plot_base_name::String`: first portion of the saved figure file name
* `plot_extension::String`: extension of saved figure files

"""
function post_plot_rotor_sweep(aircraft, parameters, step_range, step_symbol)

    # extract info
    CTs = parameters.CTs
    CQs = parameters.CQs
    ηs = parameters.ηs
    data = [CTs, CQs, ηs]
    rotor_names = parameters.rotor_names
    plot_directory = parameters.plot_directory
    plot_base_name = parameters.plot_base_name
    plot_extension = parameters.plot_extension
    nrotors = length(aircraft.rotor_system.index)

    # check data
    @assert length(rotor_names) == nrotors "length of rotor_names is inconsistent: got $(length(rotor_names)); expected $nrotors"
    @assert isdir(plot_directory) "plot_directory does not exist"

    # prepare figure
    fig = plt.figure(plot_base_name * "_rotor_sweep")
    fig.clear()
    fig.add_subplot(311, ylabel = L"C_T")
    fig.add_subplot(312, ylabel = L"C_Q")
    fig.add_subplot(313, ylabel = L"\eta", xlabel = L"J")
    axs = fig.get_axes()

    # plot
    for (i,ax) in enumerate(axs)
        for jrotor in 1:nrotors
            cratio = jrotor / nrotors
            Js = parameters.Js[jrotor,:]
            ax.plot(Js, data[i][jrotor,:], color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label = rotor_names[jrotor])
        end
    end

    axs[1].legend(loc="upper left", bbox_to_anchor=(1.01,1))
    fig.set_size_inches(7, 4, forward=true)
    fig.tight_layout()

    # save
    savepath = joinpath(plot_directory, plot_base_name * "_rotor_sweep" * plot_extension)
    fig.savefig(savepath, bbox_inches = "tight")

    # also plot power as a function of J
    fig_power = plt.figure(plot_base_name * "_rotor_power")
    fig_power.clear()
    fig_power.add_subplot(121, ylabel = L"P", xlabel = L"J")
    fig_power.add_subplot(122, xlabel = L"n")
    axs_power = fig_power.get_axes()

    # plot
    for jrotor in 1:nrotors
        cratio = jrotor / nrotors
        Js = parameters.Js[jrotor,:]
        ns = parameters.omegas[jrotor,:] ./ (2*pi)
        Ps = parameters.Ps[jrotor,:]
        axs_power[1].plot(Js, Ps, color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label = rotor_names[jrotor])
        axs_power[2].plot(ns, Ps, color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label = rotor_names[jrotor])
    end

    axs_power[2].legend(loc="upper left", bbox_to_anchor=(1.01,1))
    fig_power.set_size_inches(9, 3, forward=true)
    fig_power.tight_layout()

    # save
    savepath = joinpath(plot_directory, plot_base_name * "_rotor_power" * plot_extension)
    fig.savefig(savepath, bbox_inches = "tight")

    return false
end


"""
    post_plot_rotor_sweep(system, step_range) <: PostAction

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft`: system to be simulated
* `step_range::AbstractArray`: defines each step of the simulation

# Returns:

* `Js::AbstractArray`: array of advance ratios of the ith rotor at the jth step
* `CTs::Array{Float64,2}`: array of thrust coefficients of the ith rotor at the jth step
* `CQs::Array{Float64,2}`: array of torque coefficients of the ith rotor at the jth step
* `ηs::Array{Float64,2}`: array of propulsive efficiencies of the ith rotor at the jth step
* `rotor_names::Vector{String}`: vector of rotor names for use in plot legend
* `plot_directory::String`: directory where plots are saved
* `plot_base_name::String`: first portion of the saved figure file name
* `plot_extension::String`: extension of saved figure files

"""
function post_plot_rotor_sweep(aircraft, step_range)

    _, Js, _, _, CTs, CQs, ηs, _, _ = solve_rotor_nondimensional(aircraft, step_range)
    nrotors = length(aircraft.rotor_system.index)
    rotor_names = ["rotor$i" for i in 1:nrotors]
    plot_directory = ""
    plot_base_name = "default"
    plot_extension = ".pdf"

    return Js, CTs, CQs, ηs, rotor_names, plot_directory, plot_base_name, plot_extension
end
