#=##############################################################################################
Filename: plot_lift_moment_distribution.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: `<: Action` function plots the lift distribution of all lifting surfaces at the specified steps
=###############################################################################################

"""
    plot_lift_moment_distribution(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol) <: Action

# Arguments:

* `aircraft::Aircraft`: `Aircraft` system object
* `parameters<:Parameters`: `Parameters` struct
* `freestream::Freestream`: `Freestream` object
* `environment::Environment` `Environment` object
* `steprange::AbstractArray`: array of step for which the simulation is run
* `stepi::Int`: index of the current step
* `stepsymbol::String`: defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `surfacenames::Vector{String}`: names of each lifting surface to be shown in the legend
* `cfs::Vector{Array{Float64,2}}`: vector with length equal to the number of lifting surfaces, each member containing an array of size (3,ns) of force coefficients cd, cy, cl
* `cms::Vector{Array{Float64,2}}`: vector with length equal to the number of lifting surfaces, each member containing an array of size (3,ns) of moment coefficients cmx, cmy, cmz
* `plotdirectory::String`: path to the folder where plots will be saved
* `plotbasename::String`: first part of saved figure file names
* `plotextension::String`: extension of saved figure file names
* `plotstepi::Vector{Int}`: which steps at which to plot
"""
# * `cl_ylim::Vector{Float64}`: y axis limits for ploting c_l
# * `cd_ylim::Vector{Float64}`: y axis limits for ploting c_d
# * `cy_ylim::Vector{Float64}`: y axis limits for ploting c_y


function plot_lift_moment_distribution(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)

    # extract plot indices
    plotstepi = parameters.plotstepi
    basename = parameters.plotbasename

    # cycle through styles
    styles = ["-", "--", "--^", "--v"]

    # extract main wing span
    b = aircraft.wingsystem.system.surfaces[1][end].rtr[2] * 2

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
        fig_lift_distribution = plt.figure(basename * "_lift_distribution")
        fig_cf_distribution = plt.figure(basename * "_cf_distribution")
        nsurfaces = length(surfacenames)
        nsubplotbase = 300 + 10 * nsurfaces
        if stepi == parameters.plotstepi[1]
            fig_lift_distribution.clear()
            fig_lift_distribution.add_subplot(111, ylabel = L"c_l", xlabel = L"2y/b")
            fig_cf_distribution.clear()
            for isurface = 1:nsurfaces
                fig_cf_distribution.add_subplot(nsubplotbase + 2 + (isurface-1) * nsurfaces, ylabel = L"c_d") # drag ylim = cd_ylim,
                fig_cf_distribution.add_subplot(nsubplotbase + 3 + (isurface-1) * nsurfaces, ylabel = L"c_y", xlabel = L"y [m]") # side force ylim = cy_ylim,
                fig_cf_distribution.add_subplot(nsubplotbase + 1 + (isurface-1) * nsurfaces, ylabel = L"c_l", title = surfacenames[isurface]) # lift ylim = cl_ylim,
                # fig_cf_distribution.suptitle("t = $(steprange[stepi]), ti = $stepi") # add title

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
        ax_cl = fig_lift_distribution.get_axes()[1]
        axs_cf = fig_cf_distribution.get_axes()

        # get color
        cratio = findfirst((x)->x==stepi,plotstepi) / length(plotstepi)

        # plot
        for (isurface,cf) in enumerate(cfs)
            rs_plot = get_midpoints(lifting_line_rs[isurface][2,:])
            ax_cl.plot(rs_plot ./ b * 2, cf[3,:], styles[isurface], color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="$(surfacenames[isurface]), $stepsymbol = $(round(steprange[stepi],digits=3))")
            for icf = 1:3
                axs_cf[(isurface - 1) * nsurfaces + icf].plot(rs_plot, cf[icf,:], color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="$stepsymbol = $(round(steprange[stepi],digits=3))")
            end
        end

        # save
        if stepi == plotstepi[end] # last step
            ax_cl.legend(loc="upper left", bbox_to_anchor=(1.01,1))
            fig_lift_distribution.set_size_inches(10, 6, forward=true)
            fig_lift_distribution.tight_layout()
            savepath_cl = joinpath(plotdirectory, plotbasename * "_lift_distribution" * plotextension)
            fig_lift_distribution.savefig(savepath_cl, bbox_inches="tight")
            axs_cf[3].legend(loc="upper left", bbox_to_anchor=(1.01,1)) # set legend
            fig_cf_distribution.set_size_inches(6, 10, forward=true)
            fig_cf_distribution.tight_layout() # clean up white space
            savepath_cf = joinpath(plotdirectory, plotbasename * "_cf_distribution" * plotextension)
            fig_cf_distribution.savefig(savepath_cf, bbox_inches="tight")
        end

        # cms plot
        fig_moment_distribution = plt.figure(basename * "_moment_distribution")
        nsurfaces = length(surfacenames)
        nsubplotbase = 300 + 10 * nsurfaces
        if stepi == parameters.plotstepi[1]
            fig_moment_distribution.clear()
            for isurface = 1:nsurfaces
                fig_moment_distribution.add_subplot(nsubplotbase + 2 + (isurface-1) * nsurfaces, ylabel = L"c_{mx}") # drag ylim = cd_ylim,
                fig_moment_distribution.add_subplot(nsubplotbase + 1 + (isurface-1) * nsurfaces, ylabel = L"c_{my}", title = surfacenames[isurface]) # lift ylim = cl_ylim,
                fig_moment_distribution.add_subplot(nsubplotbase + 3 + (isurface-1) * nsurfaces, ylabel = L"c_{mz}", xlabel = L"y [m]") # side force ylim = cy_ylim,
                # fig_moment_distribution.suptitle("t = $(steprange[stepi]), ti = $stepi") # add title

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
        axs = fig_moment_distribution.get_axes()

        for (isurface,cm) in enumerate(cms)
            rs_plot = get_midpoints(lifting_line_rs[isurface][2,:])
            for icf = 1:3
                axs[(isurface - 1) * nsurfaces + icf].plot(rs_plot, cm[icf,:], color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="$stepsymbol = $(round(steprange[stepi],digits=3))")
            end
        end

        # save
        if stepi == plotstepi[end]
            axs[2].legend(loc="upper left", bbox_to_anchor=(1.01,1)) # set legend
            fig_moment_distribution.set_size_inches(6, 10, forward=true)
            fig_moment_distribution.tight_layout() # clean up white space
            savepath = joinpath(plotdirectory, plotbasename * "_moment_distribution" * plotextension)
            fig_moment_distribution.savefig(savepath, bbox_inches="tight")
        end
    end

    return false
end


# @assert length(labels) == length(aircraft.wingsystem.system.surfaces) "length of surfacenames and lifting surfaces are inconsistent"
# @assert ispath(plotdirectory) "plotdirectory does not exist"

"""
plot_lift_moment_distribution(aircraft, steprange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft`: system to be simulated
* `steprange::AbstractArray`: defines each step of the simulation

# Returns:

* `surfacenames::Vector{String}`: names of each lifting surface to be shown in the legend
* `cfs::Vector{String}`: vector with length equal to the number of lifting surfaces, each member containing an array of size (3,ns) of force coefficients
* `plotdirectory::String`: path to the folder where plots will be saved
* `plotbasename::String`: first part of saved figure file names
* `plotextension::String`: extension of saved figure file names
"""
# * `cl_ylim::Vector{Float64}`: y axis limits for ploting c_l
# * `cd_ylim::Vector{Float64}`: y axis limits for ploting c_d
# * `cy_ylim::Vector{Float64}`: y axis limits for ploting c_y

function plot_lift_moment_distribution(aircraft, steprange)

    nwings = length(aircraft.wingsystem.surfaces)
    surfacenames = Vector{String}(undef,nwings)
    cfs = [zeros(3, length(aircraft.wingsystem.surfaces[i])) for i in 1:nwings]
    cms = deepcopy(cfs)
    plotdirectory = ""
    plotbasename = "default"
    plotextension = ".pdf"
    plotstepi = deepcopy(steprange)
    # cl_ylim = zeros(2)
    # cd_ylim = zeros(2)
    # cy_ylim = zeros(2)

    return surfacenames, cfs, cms, plotdirectory, plotbasename, plotextension, plotstepi#, cl_ylim, cd_ylim, cy_ylim
end
