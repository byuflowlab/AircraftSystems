#=##############################################################################################
Filename: plot_lift_moment_distributions.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: `<: PostAction` function plots the lift distribution of all lifting surfaces at the specified steps
=###############################################################################################

"""
plot_lift_moment_distributions <: PostAction

* `aircraft::Aircraft` : aircraft system struct
* `parameters <: Parameters` : inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `steprange::AbstractArray` : range of simulation times
* `stepsymbol::String` : defines the step, e.g. `alpha` or `time`

Modifies:

* `aircraft::Aircraft`
* `parameters <: Parameters`

Outputs:

* `flag::Bool` : true if an action experiences errors
* saves plots in `parameters.plotdirectory`

`parameters <: Parameters` requires the following elements:

* `surfacenames::Vector{String}` : names of each lifting surface to be shown in the legend
* `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
* `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
* `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
* `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface
* `cfs::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl
* `cms::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy,cmz
* `plotdirectory::String` : path to the folder where plots will be saved
* `plotbasename::String` : first part of saved figure file names
* `plotextension::String` : extension of saved figure file names
* `plotstepi::Vector{Int}` : which steps at which to plot
# * `cl_ylim::Vector{Float64}` : y axis limits for ploting c_l
# * `cd_ylim::Vector{Float64}` : y axis limits for ploting c_d
# * `cy_ylim::Vector{Float64}` : y axis limits for ploting c_y

"""
function plot_lift_moment_distributions(aircraft, parameters, steprange, stepsymbol)
    # extract info
    cfs = parameters.cfs
    cms = parameters.cms
    cls = parameters.cls
    cds = parameters.cds
    cys = parameters.cys
    cmxs = parameters.cmxs
    cmys = parameters.cmys
    cmzs = parameters.cmzs
    nsurfaces = length(aircraft.wingsystem.system.surfaces)
    for stepi = 1:length(steprange) # loop over all steps
        for isurface in 1:nsurfaces
            # update cfs and cms
            cfs[isurface][:,1] .= cds[isurface][:,stepi]
            cfs[isurface][:,2] .= cys[isurface][:,stepi]
            cfs[isurface][:,3] .= cls[isurface][:,stepi]
            cms[isurface][:,1] .= cmxs[isurface][:,stepi]
            cms[isurface][:,2] .= cmys[isurface][:,stepi]
            cms[isurface][:,3] .= cmzs[isurface][:,stepi]
            # run existing plot function
            flag = plot_lift_moment_distribution(aircraft, parameters, nothing, nothing, steprange, stepi, stepsymbol)
        end
    end # loop over all steps
    return false
end


# @assert length(labels) == length(aircraft.wingsystem.system.surfaces) "length of surfacenames and lifting surfaces are inconsistent"
# @assert ispath(plotdirectory) "plotdirectory does not exist"

"""
plot_lift_moment_distributions(system, steprange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

Inputs:

* `system::System` : system to be simulated
* `steprange::AbstractArray` : defines each step of the simulation

Outputs:

* `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
* `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
* `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
* `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface
* `cfs::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl
* `cms::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy,
* `surfacenames::Vector{String}` : names of each lifting surface to be shown in the legend
* `plotdirectory::String` : path to the folder where plots will be saved
* `plotbasename::String` : first part of saved figure file names
* `plotextension::String` : extension of saved figure file names
* `plotstepi::Vector{Int}` : which steps at which to plot
# * `cl_ylim::Vector{Float64}` : y axis limits for ploting c_l
# * `cd_ylim::Vector{Float64}` : y axis limits for ploting c_d
# * `cy_ylim::Vector{Float64}` : y axis limits for ploting c_y
"""
function plot_lift_moment_distributions(aircraft, steprange)
    # extract info
    nsteps = length(steprange)
    nwings = length(aircraft.wingsystem.system.surfaces)
    nspanwisepanels = [size(surface)[2] for surface in aircraft.wingsystem.system.surfaces]
    # initialize parameters
    surfacenames = ["wing$i" for i in 1:nwings]
    cls = [zeros(nspanwisepanels[i], nsteps) for i in 1:length(surfacenames)]
    cds = deepcopy(cls)
    cys = deepcopy(cls)
    cmxs = deepcopy(cls)
    cmys = deepcopy(cls)
    cmzs = deepcopy(cls)
    # cfs = [zeros(3, length(aircraft.wingsystem.system.surfaces[i])) for i in 1:nwings]
    cfs = deepcopy(cls)
    cms = deepcopy(cls)
    plotdirectory = ""
    plotbasename = "defaultplot"
    plotextension = ".pdf"
    plotstepi = deepcopy(steprange)
    # cl_ylim = zeros(2)
    # cd_ylim = zeros(2)
    # cy_ylim = zeros(2)

    return cls, cds, cys, cmxs, cmys, cmzs, cfs, cms, surfacenames, plotdirectory, plotbasename, plotextension, plotstepi
end
