"""
    post_plot_cl_alpha_sweep(aircraft, parameters, alpharange, stepsymbol) <: PostAction

# Arguments:

* `aircraft::Aircraft`: aircraft system struct
* `parameters <: Parameters`: inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `alpharange::AbstractArray`: range of alphas in radians
* `stepsymbol::String`: defines the step, e.g. `alpha` or `time`

# Modifies:

* `aircraft::Aircraft`
* `parameters <: Parameters`

# Returns:

* `flag::Bool`: true if an action experiences errors
* saves plots in `parameters.plotdirectory`

`parameters <: Parameters` requires:

* `CLs::Vector{Float64}`: CL corresponding to the ith alphastep
* `CDs::Vector{Float64}`: CD corresponding to the ith alphastep
* `CYs::Vector{Float64}`: CY corresponding to the ith alphastep
* `plotdirectory::String`: directory where plots are saved
* `plotbasename::String`: first portion of the saved figure file name
* `plotextension::String`: extension of saved figure files

"""
function post_plot_cl_alpha_sweep(aircraft, parameters, alpharange, stepsymbol)
    
    # extract info
    CLs = parameters.CLs
    CDs = parameters.CDs
    CYs = parameters.CYs
    data = [CLs, CDs, CYs]
    plotdirectory = parameters.plotdirectory
    plotbasename = parameters.plotbasename
    plotextension = parameters.plotextension
    
    # check data
    @assert isdir(plotdirectory) "plotdirectory $plotdirectory does not exist"
    
    # prepare figure
    fig = plt.figure(plotbasename * "_cl_alpha_sweep")
    fig.clear()
    fig.add_subplot(311, ylabel = L"C_L")
    fig.add_subplot(312, ylabel = L"C_D")
    fig.add_subplot(313, ylabel = L"C_Y", xlabel = stepsymbol)
    axs = fig.get_axes()
    
    # plot
    for (i,ax) in enumerate(axs)
        ax.plot(alpharange .* 180/pi, data[i], label = "VortexLattice")
    end
    
    # save
    savepath = joinpath(plotdirectory, plotbasename * "_cl_alpha_sweep" * plotextension)
    fig.savefig(savepath, bbox_inches="tight")
    return false
end

"""
    post_plot_cl_alpha_sweep(aircraft, steprange) <: Action

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft`: aircraft system to be simulated
* `steprange::AbstractArray`: defines each step of the simulation

# Returns:

* `CLs::Vector{Float64}`: CL corresponding to the ith step
* `CDs::Vector{Float64}`: CD corresponding to the ith step
* `CYs::Vector{Float64}`: CY corresponding to the ith step
* `plotdirectory::String`: directory where plots are saved
* `plotbasename::String`: first portion of the saved figure file name
* `plotextension::String`: extension of saved figure files

"""
function post_plot_cl_alpha_sweep(aircraft, steprange)
    
    CLs = zeros(length(steprange))
    CDs = zeros(length(steprange))
    CYs = zeros(length(steprange))
    plotdirectory = ""
    plotbasename = "default"
    plotextension = ".pdf"

    return CLs, CDs, CYs, plotdirectory, plotbasename, plotextension
end
