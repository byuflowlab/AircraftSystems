#=##############################################################################################
Filename: solve_wing_CF_CM.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve a CCBlade rotor
=###############################################################################################


"""
    solve_wing_CF_CM(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

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
* `CFs::Array{Float64,2}` : [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
* `CMs::Array{Float64,2}` : [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth step

"""
function solve_wing_CF_CM(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    # interpret freestream
    vlmfreestream = VL.Freestream(freestream)

    # get reference
    reference = aircraft.wingsystem.system.reference[1]

    # solve vortex lattice
    VL.steady_analysis!(aircraft.wingsystem.system, aircraft.wingsystem.system.surfaces, aircraft.wingsystem.system.reference[1], vlmfreestream; symmetric=true, additional_velocity = parameters.wakefunctions[stepi])

    # extract forces and moments
    CF, CM = VL.body_forces(aircraft.wingsystem.system; frame=VL.Wind())

    # store to `parameters`
    parameters.CFs[:,stepi] .= CF
    parameters.CMs[:,stepi] .= CM

    return false
end


"""
solve_wing_CF_CM(system, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : aircraft system to be simulated
* `step_range::AbstractArray` : defines each step of the simulation

# Returns:

* `wakefunctions::Vector{Function}`: [i]th element is a function vwake(X::Vector{Float64}) describing the wake induced velocity at `X` at the ith step
* `CFs::Array{Float64,2}` : [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
* `CMs::Array{Float64,2}` : [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth step

"""
function solve_wing_CF_CM(aircraft, step_range)

    wakefunctions = Function[(x) -> [0.0, 0.0, 0.0] for i in 1:length(step_range)]
    CFs = zeros(3, length(step_range))
    CMs = zeros(3, length(step_range))

    return wakefunctions, CFs, CMs
end
