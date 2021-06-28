#=##############################################################################################
Filename: solve_wing_CF.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve a CCBlade rotor
=###############################################################################################


"""
    solve_wing_CF(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)

Action function.

# Arguments:

* `aircraft::Aircraft`: `Aircraft` system object
* `parameters<:Parameters`: `Parameters` struct
* `freestream::Freestream`: `Freestream` object
* `environment::Environment` `Environment` object
* `steprange::AbstractArray`: array of times for which the simulation is run
* `stepi::Int`: index of the current step
* `stepsymbol::String`: defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `wakefunctions::Vector{Function}`: [i]th element is a function vwake(X::Vector{Float64}) describing the wake induced velocity at `X` at the ith step
* `CLs::Vector{Vector{Float64}}`: a vector of length `length(steprange)` containing lift coefficients at each step
* `CDs::Vector{Vector{Float64}}`: a vector of length `length(steprange)` containing drag coefficients at each step
* `CYs::Vector{Vector{Float64}}`: a vector of length `length(steprange)` containing side force coefficients at each step

"""
function solve_wing_CF(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    
    # interpret freestream
    vlmfreestream = VL.Freestream(freestream)
    
    # get reference
    reference = aircraft.wingsystem.system.reference[1]
    
    # solve vortex lattice
    vwake = parameters.wakefunctions[stepi]
    VL.steady_analysis!(aircraft.wingsystem.system, aircraft.wingsystem.system.surfaces, aircraft.wingsystem.system.reference[1], vlmfreestream; symmetric=true, additional_velocity = vwake)
    
    # extract forces and moments
    CF, CM = VL.body_forces(aircraft.wingsystem.system; frame=VL.Wind())
    
    # store to `parameters`
    parameters.CLs[stepi] = CF[3]
    parameters.CDs[stepi] = CF[1]
    parameters.CYs[stepi] = CF[2]

    return false
end


"""
    solve_wing_CF(system, steprange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : aircraft system to be simulated
* `steprange::AbstractArray` : defines each step of the simulation

# Returns:

* `CLs::Array{Float64,1}` : ith element is the lift coefficient of the aircraft at the ith step
* `CDs::Array{Float64,1}` : ith element is the drag coefficient of the aircraft at the ith step
* `CYs::Array{Float64,1}` : ith element is the side force coefficient of the aircraft at the ith step

"""
function solve_wing_CF(aircraft, steprange)

    CLs = zeros(length(steprange))
    CDs = zeros(length(steprange))
    CYs = zeros(length(steprange))

    return CLs, CDs, CYs
end
