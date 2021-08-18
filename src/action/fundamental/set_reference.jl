#=##############################################################################################
Filename: set_wing_reference.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve a CCBlade rotor
=###############################################################################################

"""
    set_wing_reference(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

Action function.

# Arguments:

* `aircraft::Aircraft`: `Aircraft` system object
* `parameters<:Parameters`: `Parameters` struct
* `freestream::Freestream`: `Freestream` object
* `environment::Environment` `Environment` object
* `step_range::AbstractArray`: array of times for which the simulation is run
* `stepi::Int`: index of the current step
* `step_symbol::String`: defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `wakefunctions::Vector{Function}`: [i]th element is a function vwake(X::Vector{Float64}) describing the wake induced velocity at `X` at the ith step

"""
function set_wing_reference(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    # interpret freestream
    vlmfreestream = VL.Freestream(freestream)
    vwake = parameters.wakefunctions[stepi]

    # read current reference
    reference = aircraft.wingsystem.system.reference[1]
    S = reference.S
    c = reference.c
    b = reference.b
    r = reference.r
    V = reference.V

    # get new V
    newV = freestream.vinf

    # build new reference
    newreference = VL.Reference(S,c,b,r,newV)

    # set
    aircraft.wingsystem.system.reference[1] = newreference

    return false
end


"""
    set_wing_reference(aircraft, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : aircraft system to be simulated
* `step_range::AbstractArray` : defines each step of the simulation

# Returns:

* `wakefunctions::Vector{Function}`: [i]th element is a function vwake(X::Vector{Float64}) describing the wake induced velocity at `X` at the ith step

"""
function set_wing_reference(aircraft, step_range)

    wakefunctions = Function[(x) -> [0.0, 0.0, 0.0] for i in 1:length(step_range)]

    return wakefunctions
end
