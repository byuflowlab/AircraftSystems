"""
    post_plot_rotor_sweep(aircraft, parameters, steprange, stepsymbol)

# Arguments:

* `aircraft::Aircraft`: aircraft system struct
* `parameters <: Parameters`: inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `steprange::AbstractArray`: range of times defining simulation
* `stepsymbol::String`: defines the step, e.g. `alpha` or `time`

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
* `rotornames::Vector{String}`: vector of rotor names for use in plot legend
* `plotdirectory::String`: directory where plots are saved
* `plotbasename::String`: first portion of the saved figure file name
* `plotextension::String`: extension of saved figure files

"""
function post_plot_rotor_sweep(aircraft, parameters, steprange, stepsymbol)
    
    # extract info
    CTs = parameters.CTs
    CQs = parameters.CQs
    ηs = parameters.ηs
    data = [CTs, CQs, ηs]
    rotornames = parameters.rotornames
    plotdirectory = parameters.plotdirectory
    plotbasename = parameters.plotbasename
    plotextension = parameters.plotextension
    nrotors = length(aircraft.rotorsystem.index)
    
    # check data
    @assert length(rotornames) == nrotors "length of rotornames is inconsistent: got $(length(rotornames)); expected $nrotors"
    @assert isdir(plotdirectory) "plotdirectory does not exist"
    
    # prepare figure
    fig = plt.figure(plotbasename * "_rotor_sweep")
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
            ax.plot(Js, data[i][jrotor,:], color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label = rotornames[jrotor])
        end
    end

    axs[end].legend(loc="upper left", bbox_to_anchor=(1.01,1))
    fig.set_size_inches(14, 8, forward=true)
    fig.tight_layout()

    # save
    savepath = joinpath(plotdirectory, plotbasename * "_rotor_sweep" * plotextension)
    fig.savefig(savepath, bbox_inches = "tight")
    return false
end


"""
    post_plot_rotor_sweep(system, steprange) <: PostAction

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft`: system to be simulated
* `steprange::AbstractArray`: defines each step of the simulation

# Returns:

* `Js::AbstractArray`: array of advance ratios of the ith rotor at the jth step
* `CTs::Array{Float64,2}`: array of thrust coefficients of the ith rotor at the jth step
* `CQs::Array{Float64,2}`: array of torque coefficients of the ith rotor at the jth step
* `ηs::Array{Float64,2}`: array of propulsive efficiencies of the ith rotor at the jth step
* `rotornames::Vector{String}`: vector of rotor names for use in plot legend
* `plotdirectory::String`: directory where plots are saved
* `plotbasename::String`: first portion of the saved figure file name
* `plotextension::String`: extension of saved figure files

"""
function post_plot_rotor_sweep(aircraft, steprange)

    _, Js, _, _, CTs, CQs, ηs, _, _ = solve_rotor_nondimensional(aircraft, steprange)
    nrotors = length(aircraft.rotorsystem.index)
    rotornames = ["rotor$i" for i in 1:nrotors]
    plotdirectory = ""
    plotbasename = "default"
    plotextension = ".pdf"

    return Js, CTs, CQs, ηs, rotornames, plotdirectory, plotbasename, plotextension
end
