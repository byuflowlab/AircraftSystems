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

* `omegas::Vector{Float64}` : a vector of length `length(timerange)` containing a vector of rotational velocities for each rotor
* `Js_history::Vector{Vector{Float64}}` : a vector of length `length(timerange)` containing a vector of advance ratios for each rotor
* `CTs::Vector{Vector{Float64}}` : a vector of length `length(timerange)` containing a vector of thrust coefficients for each rotor
* `CQs::Vector{Vector{Float64}}` : a vector of length `length(timerange)` containing a vector of torque coefficients for each rotor
* `ηs::Vector{Vector{Float64}}` : a vector of length `length(timerange)` containing a vector of propulsive efficiencies for each rotor

"""
function solve_propeller(aircraft, parameters, freestream, environment, timerange, ti)

    omegas = parameters.omegas[ti]
    Js, CTs, CQs, ηs = solverotorsystem(aircraft.rotorsystem, omegas, freestream, environment)

    parameters.Js_history[ti] .= Js
    parameters.CTs[ti] .= CTs
    parameters.CQs[ti] .= CQs
    parameters.ηs[ti] .= ηs

    return false
end

"""
solve_propeller(system, timerange)

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
function solve_propeller(aircraft, timerange)
    nrotors = length(aircraft.rotorsystem.index) # number of rotors
    omegas = fill(ones(nrotors) .* 5000.0, length(timerange))
    Js = fill(zeros(nrotors), length(timerange))
    CTs = fill(zeros(nrotors), length(timerange))
    CQs = fill(zeros(nrotors), length(timerange))
    ηs = fill(zeros(nrotors), length(timerange))

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
