#=##############################################################################################
Filename: plot_lift_moment_distribution.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: `<: Action` function plots the lift distribution of all lifting surfaces at the specified steps
=###############################################################################################

"""
    plot_lift_moment_distribution(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol) <: Action

# Arguments:

* `aircraft::Aircraft`: `Aircraft` system object
* `parameters<:Parameters`: `Parameters` struct
* `freestream::Freestream`: `Freestream` object
* `environment::Environment` `Environment` object
* `step_range::AbstractArray`: array of step for which the simulation is run
* `stepi::Int`: index of the current step
* `step_symbol::String`: defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `surfacenames::Vector{String}`: names of each lifting surface to be shown in the legend
* `cfs::Vector{Array{Float64,2}}`: vector with length equal to the number of lifting surfaces, each member containing an array of size (3,ns) of force coefficients cd, cy, cl
* `cms::Vector{Array{Float64,2}}`: vector with length equal to the number of lifting surfaces, each member containing an array of size (3,ns) of moment coefficients cmx, cmy, cmz
* `plot_directory::String`: path to the folder where plots will be saved
* `plot_base_name::String`: first part of saved figure file names
* `plot_extension::String`: extension of saved figure file names
* `plotstepi::Vector{Int}`: which steps at which to plot
"""
function plot_lift_moment_distribution(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    # extract plot indices
    plotstepi = parameters.plotstepi
    basename = parameters.plot_base_name

    # cycle through styles
    styles = ["-", "--", "--^", "--v"]

    # extract main wing span
    b = aircraft.wing_system.system.surfaces[1][end].rtr[2] * 2

    if stepi in plotstepi

        # extract info
        surfacenames = parameters.surfacenames
        cfs = parameters.cfs
        cms = parameters.cms
        lifting_line_rs = aircraft.wing_system.lifting_line_rs
        plot_directory = parameters.plot_directory
        plot_base_name = parameters.plot_base_name
        plot_extension = parameters.plot_extension
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
                # fig_cf_distribution.suptitle("t = $(step_range[stepi]), ti = $stepi") # add title

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
        for (isurface,cf) in enumerate(cfs[stepi])
            rs_plot = get_midpoints(lifting_line_rs[isurface][2,:])
            ax_cl.plot(rs_plot ./ b * 2, cf[3,:], styles[isurface], color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="$(surfacenames[isurface]), $step_symbol = $(round(step_range[stepi],digits=3))")
            for icf = 1:3
                axs_cf[(isurface - 1) * nsurfaces + icf].plot(rs_plot, cf[icf,:], color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="$step_symbol = $(round(step_range[stepi],digits=3))")
            end
        end

        # save
        if stepi == plotstepi[end] # last step
            ax_cl.legend(loc="upper left", bbox_to_anchor=(1.01,1))
            fig_lift_distribution.set_size_inches(10, 6, forward=true)
            fig_lift_distribution.tight_layout()
            savepath_cl = joinpath(plot_directory, plot_base_name * "_lift_distribution" * plot_extension)
            fig_lift_distribution.savefig(savepath_cl, bbox_inches="tight")
            axs_cf[3].legend(loc="upper left", bbox_to_anchor=(1.01,1)) # set legend
            fig_cf_distribution.set_size_inches(6, 10, forward=true)
            fig_cf_distribution.tight_layout() # clean up white space
            savepath_cf = joinpath(plot_directory, plot_base_name * "_cf_distribution" * plot_extension)
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
                # fig_moment_distribution.suptitle("t = $(step_range[stepi]), ti = $stepi") # add title

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
        for (isurface,cm) in enumerate(cms[stepi])
            rs_plot = get_midpoints(lifting_line_rs[isurface][2,:])
            for icf = 1:3
                axs[(isurface - 1) * nsurfaces + icf].plot(rs_plot, cm[icf,:], color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="$step_symbol = $(round(step_range[stepi],digits=3))")
            end
        end

        # save
        if stepi == plotstepi[end]
            axs[2].legend(loc="upper left", bbox_to_anchor=(1.01,1)) # set legend
            fig_moment_distribution.set_size_inches(6, 10, forward=true)
            fig_moment_distribution.tight_layout() # clean up white space
            savepath = joinpath(plot_directory, plot_base_name * "_moment_distribution" * plot_extension)
            fig_moment_distribution.savefig(savepath, bbox_inches="tight")
        end
    end

    return false
end

"""
plot_lift_moment_distribution(aircraft, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft`: system to be simulated
* `step_range::AbstractArray`: defines each step of the simulation

# Returns:

* `surfacenames::Vector{String}`: names of each lifting surface to be shown in the legend
* `cfs::Vector{String}`: vector with length equal to the number of lifting surfaces, each member containing an array of size (3,ns) of force coefficients
* `plot_directory::String`: path to the folder where plots will be saved
* `plot_base_name::String`: first part of saved figure file names
* `plot_extension::String`: extension of saved figure file names
"""
function plot_lift_moment_distribution(aircraft, step_range)

    nwings = length(aircraft.wing_system.surfaces)
    surfacenames = Vector{String}(undef,nwings)
    cfs = [[zeros(3, length(aircraft.wing_system.surfaces[i])) for i in 1:nwings] for j in 1:length(step_range)]
    cms = deepcopy(cfs)
    plot_directory = ""
    plot_base_name = "default"
    plot_extension = ".pdf"
    plotstepi = deepcopy(step_range)

    return surfacenames, cfs, cms, plot_directory, plot_base_name, plot_extension, plotstepi#, cl_ylim, cd_ylim, cy_ylim
end
