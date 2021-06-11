#=##############################################################################################
Filename: solve_rotor_nondimensional.jl
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
* `steprange::AbstractArray` : array of times for which the simulation is run
* `stepi::Int` : index of the current step
* `stepsymbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `omegas::Vector{Float64}` : a vector of length `length(steprange)` containing a vector of rotational velocities for each rotor
* `Js::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing a vector of advance ratios for each rotor
* `CTs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing a vector of thrust coefficients for each rotor
* `CQs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing a vector of torque coefficients for each rotor
* `ηs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing a vector of propulsive efficiencies for each rotor

"""
function solve_rotor_nondimensional(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)

    omegas = parameters.omegas[stepi]
    Js, CTs, CQs, ηs = solverotorsystemnondimensional(aircraft.rotorsystem, omegas, freestream, environment)

    parameters.Js[:,stepi] .= Js
    parameters.CTs[:,stepi] .= CTs
    parameters.CQs[:,stepi] .= CQs
    parameters.ηs[:,stepi] .= ηs

    return false
end

"""
solve_rotor_nondimensional(system, steprange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

Inputs:

* `system::System` : system to be simulated
* `steprange::AbstractArray` : defines each step of the simulation

Outputs:

* `omegas::Vector{Float64}` : a vector of rotational speeds in rad/s at the current step
* `Js::Array{Float64,2}` : each i,jth element is the advance ratio of the ith rotor at the jth step
* `CTs::Array{Float64,2}` : each i,jth element is the thrust coefficient of the ith rotor at the jth step
* `CQs::Array{Float64,2}` : each i,jth element is the torque coefficient of the ith rotor at the jth step
* `ηs::Array{Float64,2}` : each i,jth element is the propulsive efficiency of the ith rotor at the jth step

"""
function solve_rotor_nondimensional(aircraft, steprange)
    omegas, Js, CTs, CQs, _, _ = solve_rotor(aircraft, steprange)
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
