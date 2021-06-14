"""
plot_rotorperformance <: PostAction

Inputs:

* `aircraft::Aircraft` : aircraft system struct
* `parameters <: Parameters` : inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `steprange::AbstractArray` : range of times defining simulation
* `stepsymbol::String` : defines the step, e.g. `alpha` or `time`

Modifies:

* `aircraft::Aircraft`
* `parameters <: Parameters`

Outputs:

* `flag::Bool` : true if an action experiences errors

`parameters <: Parameters` requires:

* `Js::AbstractArray` : array of advance ratios of the ith rotor at the jth step
* `CTs::Array{Float64,2}` : array of thrust coefficients of the ith rotor at the jth step
* `CQs::Array{Float64,2}` : array of torque coefficients of the ith rotor at the jth step
* `ηs::Array{Float64,2}` : array of propulsive efficiencies of the ith rotor at the jth step
* `rotornames::Vector{String}` : vector of rotor names for use in plot legend
* `plotdirectory::String` : directory where plots are saved
* `plotbasename::String` : first portion of the saved figure file name
* `plotextension::String` : extension of saved figure files

"""
function plot_rotorsweep(aircraft, parameters, steprange, stepsymbol)
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
    fig = plt.figure("rotorsweep")
    fig.clear()
    fig.add_subplot(311, ylabel = L"C_T")
    fig.add_subplot(312, ylabel = L"C_Q")
    fig.add_subplot(313, ylabel = L"\eta", xlabel = L"J")
    axs = fig.get_axes()
    # plot
    for (i,ax) in enumerate(axs)
        for jrotor in 1:nrotors
            Js = get_J(parameters, jrotor, :)
            ax.plot(Js, data[i][jrotor,:], label = rotornames[jrotor])
        end
    end
    axs[end].legend()
    # save
    savepath = joinpath(plotdirectory, plotbasename * "_rotorsweep" * plotextension)
    fig.savefig(savepath)
    return false
end
