#=##############################################################################################
Filename: solve_vlm_bem.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: `<: Action` function calculates aerodynamic forces on a blade-element momentum-vortex
        lattice system; only accounts for rotor-on-wing effects
=###############################################################################################

"""
    solve_vlm_bem(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol) <: Action

Solves for the aerodynamic forces at each step.

# Arguments:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `step_range::AbstractArray` : array of steps for which the simulation is run
* `stepi::Int` : index of the current step
* `step_symbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `omegas::Vector{Float64}` : a vector of length `length(step_range)` containing a vector of rotational velocities for each rotor
* `Js::Array{Float64,2}` : a vector of length `length(step_range)` containing a vector of advance ratios for each rotor
* `Ts::Array{Float64,2}` : a vector of length `length(step_range)` containing a vector of dimensional thrust values for each rotor
* `Qs::Array{Float64,2}` : a vector of length `length(step_range)` containing a vector of dimensional torque values for each rotor
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section
* `wake_function::Vector{Function}`: [i]th element is a function vwake(X::Vector{Float64}) describing the wake induced velocity at `X` at the ith step
* `wake_shape_functions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
* `axial_interpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `swirl_interpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `axial_multipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
* `swirl_multipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor
* `CFs::Array{Float64,2}` : [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
* `CMs::Array{Float64,2}` : [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth step
* `cfs::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl corresponding to the ith step, jth lifting surface
* `cms::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz corresponding to the ith step, jth lifting surface

"""
function solve_vlm_bem(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    flags = Vector{Bool}(undef,4)
    # solves rotors
    flags[1] = solve_rotor_nondimensional(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)
    # get wake function
    flags[2] = solve_rotor_wake(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)
    # solve VLM
    flags[3] = solve_wing_CF_CM(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)
    # extract lift and moment distribution
    flags[4] = lift_moment_distribution_blownwing(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)
    # flags[4] = lift_moment_distribution_blownwing(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    return prod(flags)
end

"""
    solve_vlm_bem(aircraft, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : system to be simulated
* `step_range::AbstractArray` : defines each step of the simulation

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
* `wake_function::Vector{Function}`: [i]th element is a function vwake(X::Vector{Float64}) describing the wake induced velocity at `X` at the ith step
* `wake_shape_functions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
* `axial_interpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `swirl_interpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `axial_multipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
* `swirl_multipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor
* `CFs::Array{Float64,2}` : [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
* `CMs::Array{Float64,2}` : [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth step
* `cfs::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl corresponding to the ith step, jth lifting surface
* `cms::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz corresponding to the ith step, jth lifting surface

"""
function solve_vlm_bem(aircraft, step_range)

    params_solve_rotor_nondimensional = solve_rotor_nondimensional(aircraft, step_range) # omegas, Js, Ts, Qs, CTs, CQs, ηs, us, vs
    params_solve_rotor_wake = solve_rotor_wake(aircraft, step_range) # wake_function, us, vs
    params_solve_wing_CF_CM = solve_wing_CF_CM(aircraft, step_range) # wake_function, CFs, CMs
    params_lift_moment_distribution = lift_moment_distribution(aircraft, step_range) # cfs, cms

    return params_solve_rotor_nondimensional..., params_solve_rotor_wake[[1,4,5,6,7,8]]..., params_solve_wing_CF_CM[2:end]..., params_lift_moment_distribution...
end
