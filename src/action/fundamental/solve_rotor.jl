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
* `steprange::AbstractArray` : array of times for which the simulation is run
* `stepi::Int` : index of the current step
* `stepsymbol::String` : defines the step, e.g. `alpha` or `time`

NOTE: `omegas` is commanded while `Js` is calculated

`parameters <: Parameters` requires the following elements:

* `omegas::Vector{Float64}` : a vector of rotational speeds in rad/s at the current step (commanded)
* `Js::Array{Float64,2}` : each i,jth element is the advance ratio of the ith rotor at the jth step (calculated)
* `Ts::Array{Float64,2}` : each i,jth element is the thrust of the ith rotor at the jth step
* `Qs::Array{Float64,2}` : each i,jth element is the torque of the ith rotor at the jth step
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section

"""
function solve_rotor(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    # extract parameters
    omegas = parameters.omegas[stepi]
    Js = parameters.Js[:,stepi]
    Ts = parameters.Ts[:,stepi]
    Qs = parameters.Qs[:,stepi]
    us = parameters.us[stepi]
    vs = parameters.vs[stepi]
    solverotors!(Js, Ts, Qs, us, vs, aircraft.rotorsystem, omegas, freestream, environment)

    return false
end

"""
solve_rotor(system, steprange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

Inputs:

* `aircraft::Aircraft` : system to be simulated
* `steprange::AbstractArray` : defines each step of the simulation

Outputs:

* `omegas::Array{Float64,2}` : [i,j]th elemnt is the rotational speeds in rad/s of the ith rotor at the jth step
* `Js::Array{Float64,2}` : [i,j]th element is the advance ratio of the ith rotor at the jth step
* `Ts::Array{Float64,2}` : [i,j]th element is the thrust of the ith rotor at the jth step
* `Qs::Array{Float64,2}` : [i,j]th element is the torque of the ith rotor at the jth step
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section

"""
function solve_rotor(aircraft, steprange)
    nrotors = length(aircraft.rotorsystem.index) # number of rotors
    omegas = ones(Float64, nrotors, length(steprange))
    Js = zeros(nrotors, length(steprange))
    Ts = zeros(nrotors, length(steprange))
    Qs = zeros(nrotors, length(steprange))
    us = [[zeros(length(aircraft.rotorsystem.rlists[i])) for i in aircraft.rotorsystem.index] for _ in 1:length(steprange)]
    vs = deepcopy(us)

    return omegas, Js, Ts, Qs, us, vs
end

# # prepare solutionkeys and solutioninits for this particular system and simulation
# solutionkeys = [
#     "thrust",
#     "torque",
#     "efficiency",
#     "u",
#     "v"
# ]
