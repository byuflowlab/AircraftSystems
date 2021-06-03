#=##############################################################################################
Filename: solve_rotor.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve a CCBlade rotor
=###############################################################################################

"""
Action function.

Inputs:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `timerange::AbstractArray` : array of times for which the simulation is run
* `ti::Int` : index of the current timestep

`parameters <: Parameters` requires the following elements:

* `reference::VL.Reference` : a VortexLattice `Reference` structs to be used with each angle of attack
* `alphas::Vector{Float64}` : a vector of length `length(timerange)` containing a vector of angles of attack over which to sweep
* `CLs::Vector{Vector{Float64}}` : a vector of length `length(timerange)` containing a vector of lift coefficients for each lifting surface in `aircraft.wingsystem.system`
* `CDs::Vector{Vector{Float64}}` : a vector of length `length(timerange)` containing a vector of drag coefficients for each lifting surface in `aircraft.wingsystem.system`
* `CSs::Vector{Vector{Float64}}` : a vector of length `length(timerange)` containing a vector of side force coefficients for each lifting surface in `aircraft.wingsystem.system`

"""
function solve_wing(aircraft, parameters, freestream, environment, timerange, ti)
    # interpret freestream
    vlmfreestream = VL.Freestream(freestream)
    # get reference
    reference = parameters.reference
    # solve vortex lattice
    VL.steady_analysis!(aircraft.wingsystem.system, aircraft.wingsystem.system.surfaces, reference, vlmfreestream; symmetric=true)#, additional_velocity = vwake)
    # extract forces and moments
    CF, CM = VL.body_forces(aircraft.wingsystem.system; frame=VL.Wind())
    # store to `parameters`
    parameters.CLs[ti] .= CF[3]
    parameters.CDs[ti] .= CF[1]
    parameters.CSs[ti] .= CF[2]

    return false
end

"""
solve_wing(system, timerange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

Inputs:

* `system::System` : system to be simulated
* `timerange::AbstractArray` : defines each time step of the simulation

Outputs:

* `omegas::Vector{Float64}` : a vector of rotational speeds in rad/s at the current timestep
* `Js::Vector{Vector{Float64}}` : each ith element is a vector of advance ratios corresponding to each rotor of `system.rotors` at the ith timestep
* `CTs::Vector{Vector{Float64}}` : each ith element is a vector of thrust coefficients corresponding to each rotor of `system.rotors` at the ith timestep
* `CQs::Vector{Vector{Float64}}` : each ith element is a vector of torque coefficients corresponding to each rotor of `system.rotors` at the ith timestep
* `ηs::Vector{Vector{Float64}}` : each ith element is a vector of propulsive efficiencies corresponding to each rotor of `system.rotors` at the ith timestep

"""
function solve_wing(aircraft, timerange)
    unitreference = VL.Reference(1.0,1.0,1.0,1.0,1.0)

    nwings = length(aircraft.wingsystem.system.surfaces) # number of rotors
    alphas = range(-5.0, stop=10.0, length=length(timerange))

    CLs = fill(zeros(nwings), length(timerange))
    CDs = fill(zeros(nwings), length(timerange))
    CSs = fill(zeros(nwings), length(timerange))

    return omegas, Js, CTs, CQs, ηs
end

# # prepare solutionkeys and solutioninits for this particular system and simulation
# solutionkeys = [
#     "thrust",
#     "torque",
#     "efficiency",
#     "u",
#     "v"
# ]
