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

* `wake_function::Function` : a function describing the induced velocity behind each propeller as a function of global coordinates
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section
* `wake_shape_functions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
* `axial_interpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `swirl_interpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `axial_multipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
* `swirl_multipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor

"""
function solve_rotor_wake(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    us = parameters.us[stepi]
    vs = parameters.vs[stepi]
    wake_shape_functions = parameters.wake_shape_functions
    axial_interpolations = parameters.axial_interpolations
    swirl_interpolations = parameters.swirl_interpolations
    axial_multipliers = parameters.axial_multipliers
    swirl_multipliers = parameters.swirl_multipliers
    wakefunction = induced2wakefunction(aircraft.rotor_system, us, vs;
        wake_shape_functions,
        axial_interpolations,
        swirl_interpolations,
        axial_multipliers,
        swirl_multipliers
    )

    parameters.wake_function[stepi] = wakefunction

    return false
end


"""
    solve_rotor_wake(aircraft, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : system to be simulated
* `step_range::AbstractArray` : defines each step of the simulation

# Returns:

* `wake_function::Vector{Union{Function, Nothing}}` : [i]th element is a function describing the velocity field of rotor wakes as a function of global coordinates at the ith step
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section
* `wake_shape_functions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
* `axial_interpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `swirl_interpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `axial_multipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
* `swirl_multipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor

"""
function solve_rotor_wake(aircraft, step_range)

    wake_function = Function[(x) -> [0.0, 0.0, 0.0] for i in 1:length(step_range)]
    _, _, _, _, us, vs = solve_rotor(aircraft, step_range)

    wake_shape_functions = Function[(Rtip, x) -> Rtip for i in 1:length(aircraft.rotor_system.index)]
    axial_interpolations = Function[(rs, us, r, Rtip) -> FM.linear(rs, us, r) for i in 1:length(aircraft.rotor_system.index)]
    swirl_interpolations = Function[(rs, vs, r, Rtip) -> FM.linear(rs, vs, r) for i in 1:length(aircraft.rotor_system.index)]
    axial_multipliers = Function[(distance2plane, Rtip) -> 2 for i in 1:length(aircraft.rotor_system.index)]
    swirl_multipliers = Function[(distance2plane, Rtip) -> 1 for i in 1:length(aircraft.rotor_system.index)]

    return wake_function, us, vs, wake_shape_functions, axial_interpolations, swirl_interpolations, axial_multipliers, swirl_multipliers
end

# # prepare solutionkeys and solutioninits for this particular system and simulation
# solutionkeys = [
#     "thrust",
#     "torque",
#     "efficiency",
#     "u",
#     "v"
# ]
