#=##############################################################################################
Filename: solve_wing_viscous_drag.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve a CCBlade rotor
=###############################################################################################

"""
    solve_wing_viscous_drag <: Action

Inputs:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `step_range::AbstractArray` : array of times for which the simulation is run
* `stepi::Int` : index of the current step
* `step_symbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
* `wing_strip_ys::Vector{Vector{Float64}}` : [i][j]th element is the jth dimensional spanwise coordinate coinciding with control points of the lifting line of the ith lifting surface
* `wing_strip_polars Vector{Vector{<:CCBlade.AFType}}` : [i][j]th element is a `<: CCBlade.AFType` object corresponding to the jth lifting line section of the ith lifting surface
* `cds_viscous::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing viscous-corrected local drag coefficients at each lifting line section, corresponding to each lifting surface
* `CDs_viscous::Array{Float64,1}` : ith element is the drag coefficient of the aircraft at the ith step

"""
function solve_wing_viscous_drag(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)
    # extract parameters
    cls = parameters.cls
    cds = parameters.cds
    wing_strip_ys = parameters.wing_strip_ys
    wing_strip_polars = parameters.wing_strip_polars
    # apply viscous correction
    nwings = length(aircraft.wingsystem.system.surfaces)
    for iwing in 1:nwings

    end
    # update cds of aircraft.wingsystem.system

    # extract forces and moments
    CF_viscous, CM_viscous = VL.body_forces(aircraft.wingsystem.system; frame=VL.Wind())
    # store to `parameters`
    for iwing in 1:nwings
        parameters.cds[iwing][:,stepi] = cfs[iwing][1,:]
        parameters.cmxs[iwing][:,stepi] = cms[iwing][1,:]
        parameters.cmys[iwing][:,stepi] = cms[iwing][2,:]
        parameters.cmzs[iwing][:,stepi] = cms[iwing][3,:]
    end
    parameters.cds_viscous[stepi] .= cds_viscous
    parameters.CDs_viscous[stepi] = CF_viscous[1]

    return false
end

"""
solve_wing_viscous_drag(system, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

Inputs:

* `aircraft::Aircraft` : aircraft system to be simulated
* `step_range::AbstractArray` : defines each step of the simulation

Outputs:

* `CLs::Array{Float64,1}` : ith element is the lift coefficient of the aircraft at the ith step
* `CDs::Array{Float64,1}` : ith element is the drag coefficient of the aircraft at the ith step
* `CYs::Array{Float64,1}` : ith element is the side force coefficient of the aircraft at the ith step

"""
function solve_wing_viscous_drag(aircraft, step_range)

    CLs = zeros(length(step_range))
    CDs = zeros(length(step_range))
    CYs = zeros(length(step_range))

    return CLs, CDs, CYs
end
