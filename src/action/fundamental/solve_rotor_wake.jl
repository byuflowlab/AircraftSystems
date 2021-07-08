#=##############################################################################################
Filename: solve_rotor_wake.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve for the wake of a rotor system
=###############################################################################################


"""
    solve_rotor_wake(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol) <: Action

# Arguments:

* `aircraft::Aircraft`: `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream`: `Freestream` object
* `environment::Environment` `Environment` object
* `steprange::AbstractArray`: array of times for which the simulation is run
* `stepi::Int`: index of the current step
* `stepsymbol::String`: defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `wakefunctions::Function`: a function describing the induced velocity behind each propeller as a function of global coordinates
* `us::Vector{Vector{Vector{Float64}}}`: each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}`: each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section

"""
function solve_rotor_wake(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)

    us = parameters.us[stepi]
    vs = parameters.vs[stepi]
    wakefunction = induced2wakefunction(aircraft.rotorsystem, us, vs;
                                        wakeshapefunction = (Rtip, x) -> Rtip,
                                        axialinterpolation = (rs, us, r) -> FM.linear(rs, us, r),
                                        swirlinterpolation = (rs, vs, r) -> FM.linear(rs, vs, r),
                                        axialmultiplier = (distance2plane, Rtip) -> 2,
                                        swirlmultiplier = (distance2plane, Rtip) -> -1
                                        )

    parameters.wakefunctions[stepi] = wakefunction

    return false
end


"""
    solve_rotor_wake(aircraft, steprange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : system to be simulated
* `steprange::AbstractArray` : defines each step of the simulation

# Returns:

* `wakefunctions::Union{Function, Nothing}` : a function describing the induced velocity behind each propeller as a function of global coordinates
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section

"""
function solve_rotor_wake(aircraft, steprange)
    
    wakefunctions = Vector{Any}(nothing,length(steprange))
    _, _, _, _, us, vs = solve_rotor(aircraft, steprange)

    return wakefunctions, us, vs
end

# # prepare solutionkeys and solutioninits for this particular system and simulation
# solutionkeys = [
#     "thrust",
#     "torque",
#     "efficiency",
#     "u",
#     "v"
# ]
