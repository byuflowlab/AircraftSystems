#=##############################################################################################
Filename: solve_rotor_wake.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve for the wake of a rotor system
=###############################################################################################


"""
    solve_rotor_wake(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol) <: Action

# Arguments:

* `aircraft::Aircraft`: `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream`: `Freestream` object
* `environment::Environment` `Environment` object
* `step_range::AbstractArray`: array of times for which the simulation is run
* `stepi::Int`: index of the current step
* `step_symbol::String`: defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `wakefunctions::Function` : a function describing the induced velocity behind each propeller as a function of global coordinates
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section
* `wakeshapefunctions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
* `axialinterpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `swirlinterpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `axialmultipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
* `swirlmultipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor

"""
function solve_rotor_wake(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    us = parameters.us[stepi]
    vs = parameters.vs[stepi]
    wakeshapefunctions = parameters.wakeshapefunctions
    axialinterpolations = parameters.axialinterpolations
    swirlinterpolations = parameters.swirlinterpolations
    axialmultipliers = parameters.axialmultipliers
    swirlmultipliers = parameters.swirlmultipliers
    wakefunction = induced2wakefunction(aircraft.rotorsystem, us, vs;
        wakeshapefunctions,
        axialinterpolations,
        swirlinterpolations,
        axialmultipliers,
        swirlmultipliers
    )

    parameters.wakefunctions[stepi] = wakefunction

    return false
end


"""
    solve_rotor_wake(aircraft, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : system to be simulated
* `step_range::AbstractArray` : defines each step of the simulation

# Returns:

* `wakefunctions::Union{Function, Nothing}` : a function describing the induced velocity behind each propeller as a function of global coordinates
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section
* `wakeshapefunctions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
* `axialinterpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `swirlinterpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `axialmultipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
* `swirlmultipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor

"""
function solve_rotor_wake(aircraft, step_range)

    wakefunctions = Function[(x) -> [0.0, 0.0, 0.0] for i in 1:length(step_range)]
    _, _, _, _, us, vs = solve_rotor(aircraft, step_range)

    wakeshapefunctions = Function[(Rtip, x) -> Rtip for i in 1:length(aircraft.rotorsystem.index)]
    axialinterpolations = Function[(rs, us, r, Rtip) -> FM.linear(rs, us, r) for i in 1:length(aircraft.rotorsystem.index)]
    swirlinterpolations = Function[(rs, vs, r, Rtip) -> FM.linear(rs, vs, r) for i in 1:length(aircraft.rotorsystem.index)]
    axialmultipliers = Function[(distance2plane, Rtip) -> 2 for i in 1:length(aircraft.rotorsystem.index)]
    swirlmultipliers = Function[(distance2plane, Rtip) -> 1 for i in 1:length(aircraft.rotorsystem.index)]

    return wakefunctions, us, vs, wakeshapefunctions, axialinterpolations, swirlinterpolations, axialmultipliers, swirlmultipliers
end

# # prepare solutionkeys and solutioninits for this particular system and simulation
# solutionkeys = [
#     "thrust",
#     "torque",
#     "efficiency",
#     "u",
#     "v"
# ]
