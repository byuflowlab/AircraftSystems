#=##############################################################################################
Filename: solve_rotor_nondimensional.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve a CCBlade rotor
=###############################################################################################

"""
    solve_rotor_nondimensional(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol) <: Action

# Arguments:

* `aircraft::Aircraft`: `Aircraft` system object
* `parameters<:Parameters`: `Parameters` struct
* `freestream::Freestream`: `Freestream` object
* `environment::Environment` `Environment` object
* `steprange::AbstractArray`: array of times for which the simulation is run
* `stepi::Int`: index of the current step
* `stepsymbol::String`: defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `omegas::Vector{Float64}`: a vector of length `length(steprange)` containing a vector of rotational velocities for each rotor
* `Js::Vector{Vector{Float64}}`: a vector of length `length(steprange)` containing a vector of advance ratios for each rotor
* `Ts::Array{Float64,2}`: each [i,j]th element is the thrust of the ith rotor at the jth step
* `Qs::Array{Float64,2}`: each [i,j]th element is the torque of the ith rotor at the jth step
* `CTs::Array{Float64,2}`: each [i,j]th element is the thrust coefficient of the ith rotor at the jth step
* `CQs::Array{Float64,2}`: each [i,j]th element is the torque coefficient of the ith rotor at the jth step
* `ηs::Array{Float64,2}`: each [i,j]th element is the propulsive efficiency of the ith rotor at the jth step

"""
function solve_rotor_nondimensional(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    
    Js = view(parameters.Js,:,stepi)
    Ts = view(parameters.Ts,:,stepi)
    Qs = view(parameters.Qs,:,stepi)
    CTs = view(parameters.CTs,:,stepi)
    CQs = view(parameters.CQs,:,stepi)
    ηs = view(parameters.ηs,:,stepi)
    us = parameters.us[stepi]
    vs = parameters.vs[stepi]
    omegas = parameters.omegas[stepi]

    # println("Sherlock!\n\tBEFORE:\n\n\t\tJs = $Js\n\n\t\tTs = $Ts\n\n\t\tCTs = $CTs\n\n\t\tomegas = $omegas")

    solverotorsnondimensional!(Js, Ts, Qs, CTs, CQs, ηs, us, vs, aircraft.rotorsystem, omegas, freestream, environment)

    # println("\n\tAFTER:\n\n\t\tJs = $Js\n\n\t\tTs = $Ts\n\n\t\tCTs = $CTs\n\n\t\tomegas = $omegas")

    return false
end

"""
    solve_rotor_nondimensional(aircraft, steprange) <: Action

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : system to be simulated
* `steprange::AbstractArray` : defines each step of the simulation

# Returns:

* `omegas::Vector{Float64}` : a vector of rotational speeds in rad/s at the current step
* `Js::Array{Float64,2}` : each [i,j]th element is the advance ratio of the ith rotor at the jth step
* `Ts::Array{Float64,2}` : each [i,j]th element is the thrust of the ith rotor at the jth step
* `Qs::Array{Float64,2}` : each [i,j]th element is the torque of the ith rotor at the jth step
* `CTs::Array{Float64,2}` : each [i,j]th element is the thrust coefficient of the ith rotor at the jth step
* `CQs::Array{Float64,2}` : each [i,j]th element is the torque coefficient of the ith rotor at the jth step
* `ηs::Array{Float64,2}` : each [i,j]th element is the propulsive efficiency of the ith rotor at the jth step
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section

"""
function solve_rotor_nondimensional(aircraft, steprange)
    
    omegas, Js, Ts, Qs, us, vs = solve_rotor(aircraft, steprange)
    CTs = deepcopy(Qs)
    CQs = deepcopy(Qs)
    ηs = deepcopy(Qs)

    return omegas, Js, Ts, Qs, CTs, CQs, ηs, us, vs
end
