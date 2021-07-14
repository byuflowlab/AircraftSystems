#=##############################################################################################
Filename: plot_lift_moment_distribution.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: `<: Action` function plots the lift distribution of all lifting surfaces at the specified steps
=###############################################################################################

"""
plot_lift_moment_distribution <: Action

Inputs:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `steprange::AbstractArray` : array of step for which the simulation is run
* `stepi::Int` : index of the current step
* `stepsymbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `surfacenames::Vector{String}` : names of each lifting surface to be shown in the legend
* `cfs::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,ns) of force coefficients cd, cy, cl
* `cms::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,ns) of moment coefficients cmx, cmy, cmz
* `plotdirectory::String` : path to the folder where plots will be saved
* `plotbasename::String` : first part of saved figure file names
* `plotextension::String` : extension of saved figure file names
* `plotstepi::Vector{Int}` : which steps at which to plot
# * `cl_ylim::Vector{Float64}` : y axis limits for ploting c_l
# * `cd_ylim::Vector{Float64}` : y axis limits for ploting c_d
# * `cy_ylim::Vector{Float64}` : y axis limits for ploting c_y

"""
function plot_lift_moment_distribution(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    # extract plot indices
    plotstepi = parameters.plotstepi
    basename = parameters.plotbasename
    if stepi in plotstepi
        # extract info
        surfacenames = parameters.surfacenames
        cfs = parameters.cfs
        cms = parameters.cms
        lifting_line_rs = aircraft.wingsystem.lifting_line_rs
        plotdirectory = parameters.plotdirectory
        plotbasename = parameters.plotbasename
        plotextension = parameters.plotextension
        # cl_ylim = parameters.cl_ylim
        # cd_ylim = parameters.cd_ylim
        # cy_ylim = parameters.cy_ylim

        # create axes
        fig = plt.figure(basename * "_liftdistribution")
        nsurfaces = length(surfacenames)
        nsubplotbase = 300 + 10 * nsurfaces
        if stepi == parameters.plotstepi[1]
            fig.clear()
            for isurface = 1:nsurfaces
                fig.add_subplot(nsubplotbase + 2 + (isurface-1) * nsurfaces, ylabel = L"c_d") # drag ylim = cd_ylim,
                fig.add_subplot(nsubplotbase + 3 + (isurface-1) * nsurfaces, ylabel = L"c_y", xlabel = L"y [m]") # side force ylim = cy_ylim,
                fig.add_subplot(nsubplotbase + 1 + (isurface-1) * nsurfaces, ylabel = L"c_l", title = surfacenames[isurface]) # lift ylim = cl_ylim,
                # fig.suptitle("t = $(steprange[stepi]), ti = $stepi") # add title
                # set axes
                # axs[1].set_ylim(cd_ylim)
                # axs[1].set_ylabel(L"c_d")
                # # axs[1].legend()
                # axs[2].set_ylim(cy_ylim)
                # axs[2].set_ylabel(L"c_y")
                # axs[2].set_xlabel(L"y [m]")
                # axs[2].legend()
                # axs[3].set_ylim(cl_ylim)
                # axs[3].set_ylabel(L"c_l")
            end
        end

        # get color
        cratio = findfirst((x)->x==stepi,plotstepi) / length(plotstepi)

        # plot
        axs = fig.get_axes()
        for (isurface,cf) in enumerate(cfs)
            rs_plot = get_midpoints(lifting_line_rs[isurface][2,:])
            for icf = 1:3
                axs[(isurface - 1) * nsurfaces + icf].plot(rs_plot, cf[icf,:], color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="$stepsymbol = $(round(steprange[stepi],digits=3))")
            end
        end

        # save
        if stepi == plotstepi[end] # last step
            axs[3].legend(loc="upper left", bbox_to_anchor=(1.01,1)) # set legend
            fig.tight_layout() # clean up white space
            savepath = joinpath(plotdirectory, plotbasename * "_liftdistribution" * plotextension)
            fig.savefig(savepath, bbox_inches="tight")
        end

        # cms plot
        fig = plt.figure(basename * "_momentdistribution")
        nsurfaces = length(surfacenames)
        nsubplotbase = 300 + 10 * nsurfaces
        if stepi == parameters.plotstepi[1]
            fig.clear()
            for isurface = 1:nsurfaces
                fig.add_subplot(nsubplotbase + 2 + (isurface-1) * nsurfaces, ylabel = L"c_{mx}") # drag ylim = cd_ylim,
                fig.add_subplot(nsubplotbase + 1 + (isurface-1) * nsurfaces, ylabel = L"c_{my}", title = surfacenames[isurface]) # lift ylim = cl_ylim,
                fig.add_subplot(nsubplotbase + 3 + (isurface-1) * nsurfaces, ylabel = L"c_{mz}", xlabel = L"y [m]") # side force ylim = cy_ylim,
                # fig.suptitle("t = $(steprange[stepi]), ti = $stepi") # add title
                # set axes
                # axs[1].set_ylim(cd_ylim)
                # axs[1].set_ylabel(L"c_d")
                # # axs[1].legend()
                # axs[2].set_ylim(cy_ylim)
                # axs[2].set_ylabel(L"c_y")
                # axs[2].set_xlabel(L"y [m]")
                # axs[2].legend()
                # axs[3].set_ylim(cl_ylim)
                # axs[3].set_ylabel(L"c_l")
            end
        end

        # plot
        axs = fig.get_axes()
        for (isurface,cm) in enumerate(cms)
            rs_plot = get_midpoints(lifting_line_rs[isurface][2,:])
            for icf = 1:3
                axs[(isurface - 1) * nsurfaces + icf].plot(rs_plot, cm[icf,:], color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="$stepsymbol = $(round(steprange[stepi],digits=3))")
            end
        end

        # save
        if stepi == plotstepi[end]
            axs[2].legend(loc="upper left", bbox_to_anchor=(1.01,1)) # set legend
            fig.tight_layout() # clean up white space
            savepath = joinpath(plotdirectory, plotbasename * "_momentdistribution" * plotextension)
            fig.savefig(savepath, bbox_inches="tight")
        end
    end

    return false
end


# @assert length(labels) == length(aircraft.wingsystem.system.surfaces) "length of surfacenames and lifting surfaces are inconsistent"
# @assert ispath(plotdirectory) "plotdirectory does not exist"

"""
plot_lift_moment_distribution(system, steprange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

Inputs:

* `aircraft::Aircraft` : system to be simulated
* `steprange::AbstractArray` : defines each step of the simulation

Outputs:

* `surfacenames::Vector{String}` : names of each lifting surface to be shown in the legend
* `cfs::Vector{String}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,ns) of force coefficients
* `plotdirectory::String` : path to the folder where plots will be saved
* `plotbasename::String` : first part of saved figure file names
* `plotextension::String` : extension of saved figure file names
# * `cl_ylim::Vector{Float64}` : y axis limits for ploting c_l
# * `cd_ylim::Vector{Float64}` : y axis limits for ploting c_d
# * `cy_ylim::Vector{Float64}` : y axis limits for ploting c_y

"""
function plot_lift_moment_distribution(aircraft, steprange)

    nwings = length(aircraft.wingsystem.surfaces)
    surfacenames = Vector{String}(undef,nwings)
    cfs = [zeros(3, length(aircraft.wingsystem.surfaces[i])) for i in 1:nwings]
    cms = deepcopy(cfs)
    plotdirectory = ""
    plotbasename = "defaultplot"
    plotextension = ".pdf"
    plotstepi = deepcopy(steprange)
    # cl_ylim = zeros(2)
    # cd_ylim = zeros(2)
    # cy_ylim = zeros(2)

    return surfacenames, cfs, cms, plotdirectory, plotbasename, plotextension, plotstepi#, cl_ylim, cd_ylim, cy_ylim
end
