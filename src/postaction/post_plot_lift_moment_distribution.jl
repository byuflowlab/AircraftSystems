#=##############################################################################################
Filename: post_plot_lift_moment_distribution.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: `<: PostAction` function plots the lift distribution of all lifting surfaces at the specified steps
=###############################################################################################


"""
    post_plot_lift_moment_distribution(aircraft, parameters, step_range, step_symbol) <: PostAction

# Arguments

* `aircraft::Aircraft`: aircraft system struct
* `parameters <: Parameters`: inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `step_range::AbstractArray`: range of simulation times
* `step_symbol::String`: defines the step, e.g. `alpha` or `time`

# Modifies:

* `aircraft::Aircraft`
* `parameters <: Parameters`

# Returns:

* `flag::Bool`: true if an action experiences errors
* saves plots in `parameters.plot_directory`

`parameters <: Parameters` requires the following elements:

* `surfacenames::Vector{String}`: names of each lifting surface to be shown in the legend
* `cfs::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl corresponding to the ith step, jth lifting surface
* `cms::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz corresponding to the ith step, jth lifting surface
* `plot_directory::String`: path to the folder where plots will be saved
* `plot_base_name::String`: first part of saved figure file names
* `plot_extension::String`: extension of saved figure file names
* `plotstepi::Vector{Int}`: which steps at which to plot

"""
# * `cl_ylim::Vector{Float64}`: y axis limits for ploting c_l
# * `cd_ylim::Vector{Float64}`: y axis limits for ploting c_d
# * `cy_ylim::Vector{Float64}`: y axis limits for ploting c_y

function post_plot_lift_moment_distribution(aircraft, parameters, step_range, step_symbol)

    flags = Vector{Bool}(undef,length(step_range))
    for stepi = 1:length(step_range) # loop over all steps
        # run existing plot function
        flags[stepi] = plot_lift_moment_distribution(aircraft, parameters, nothing, nothing, step_range, stepi, step_symbol)
    end # loop over all steps

    return prod(flags)
end

"""
    post_plot_lift_moment_distribution(system, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft`: system to be simulated
* `step_range::AbstractArray`: defines each step of the simulation

# Returns:

* `cfs::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl corresponding to the ith step, jth lifting surface
* `cms::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz corresponding to the ith step, jth lifting surface
* `surfacenames::Vector{String}`: names of each lifting surface to be shown in the legend
* `plot_directory::String`: path to the folder where plots will be saved
* `plot_base_name::String`: first part of saved figure file names
* `plot_extension::String`: extension of saved figure file names
* `plotstepi::Vector{Int}`: which steps at which to plot

"""
function post_plot_lift_moment_distribution(aircraft, step_range)

    # extract info
    nsteps = length(step_range)
    nwings = length(aircraft.wing_system.system.surfaces)
    # initialize parameters
    surfacenames = ["wing$i" for i in 1:nwings]
    cfs = [[zeros(3, length(aircraft.wing_system.system.surfaces[i])) for i in 1:nwings] for j in 1:nsteps]
    cms = deepcopy(cfs)
    plot_directory = ""
    plot_base_name = "default"
    plot_extension = ".pdf"
    plotstepi = deepcopy(step_range)

    return cfs, cms, surfacenames, plot_directory, plot_base_name, plot_extension, plotstepi
end
