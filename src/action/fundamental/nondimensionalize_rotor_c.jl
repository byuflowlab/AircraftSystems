#=##############################################################################################
Filename: nondimensionalize_rotor_c.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve a CCBlade rotor
=###############################################################################################

"""
    nondimensionalize_rotor_c(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol) <: Action

# Arguments:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `step_range::AbstractArray` : array of times for which the simulation is run
* `stepi::Int` : index of the current step
* `step_symbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `omegas::Vector{Float64}` : a vector of length `length(step_range)` containing a vector of rotational velocities for each rotor
* `Js::Vector{Vector{Float64}}` : a vector of length `length(step_range)` containing a vector of advance ratios for each rotor
* `CTs::Vector{Vector{Float64}}` : a vector of length `length(step_range)` containing a vector of thrust coefficients for each rotor
* `CQs::Vector{Vector{Float64}}` : a vector of length `length(step_range)` containing a vector of torque coefficients for each rotor
* `ηs::Vector{Vector{Float64}}` : a vector of length `length(step_range)` containing a vector of propulsive efficiencies for each rotor

"""
function nondimensionalize_rotor_c(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    omegas = parameters.omegas[stepi]
    Js, CTs, CQs, ηs = solverotorsnondimensional(aircraft.rotor_system, omegas, freestream, environment)

    parameters.Js[:,stepi] .= Js
    parameters.CTs[:,stepi] .= CTs
    parameters.CQs[:,stepi] .= CQs
    parameters.ηs[:,stepi] .= ηs

    return false
end

"""
    nondimensionalize_rotor_c(aircraft, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : system to be simulated
* `step_range::AbstractArray` : defines each step of the simulation

# Returns:

* `omegas::Vector{Float64}` : a vector of rotational speeds in rad/s at the current step
* `Js::Array{Float64,2}` : each i,jth element is the advance ratio of the ith rotor at the jth step
* `CTs::Array{Float64,2}` : each i,jth element is the thrust coefficient of the ith rotor at the jth step
* `CQs::Array{Float64,2}` : each i,jth element is the torque coefficient of the ith rotor at the jth step
* `ηs::Array{Float64,2}` : each i,jth element is the propulsive efficiency of the ith rotor at the jth step

"""
function nondimensionalize_rotor_c(aircraft, step_range)

    omegas, Js, CTs, CQs, _, _ = solve_rotor(aircraft, step_range)
    ηs = deepcopy(CQs)

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
