#=##############################################################################################
Filename: solve_vlm_bem.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: `<: Action` function calculates aerodynamic forces on a blade-element momentum-vortex
        lattice system; only accounts for rotor-on-wing effects
=###############################################################################################

"""
solve_vlm_bem <: Action

Solves for the aerodynamic forces at each step.

Inputs:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `steprange::AbstractArray` : array of steps for which the simulation is run
* `stepi::Int` : index of the current step
* `stepsymbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `omegas::Vector{Float64}` : a vector of length `length(steprange)` containing a vector of rotational velocities for each rotor
* `Js::Array{Float64,2}` : a vector of length `length(steprange)` containing a vector of advance ratios for each rotor
* `Ts::Array{Float64,2}` : a vector of length `length(steprange)` containing a vector of dimensional thrust values for each rotor
* `Qs::Array{Float64,2}` : a vector of length `length(steprange)` containing a vector of dimensional torque values for each rotor
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section
* `wakefunction::Function` : function accepts position X and returns the rotor induced velocity
* `wakeshapefunctions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
* `axialinterpolation::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `swirlinterpolation::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `axialmultiplier::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
* `swirlmultiplier::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor
* `CLs::Vector{Float64}` : a vector of length `length(steprange)` containing lift coefficients at each step
* `CDs::Vector{Float64}` : a vector of length `length(steprange)` containing drag coefficients at each step
* `CYs::Vector{Float64}` : a vector of length `length(steprange)` containing side force coefficients at each step
* `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
* `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
* `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
* `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface

"""
function solve_vlm_bem(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    flags = Vector{Bool}(undef,4)
    # solves rotors
    flags[1] = solve_rotor_nondimensional(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    # get wake function
    flags[2] = solve_rotor_wake(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    # solve VLM
    flags[3] = solve_wing_CF(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    # extract lift and moment distribution
    flags[4] = lift_moment_distribution(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)

    return prod(flags)
end

"""
solve_vlm_bem(system, steprange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

Inputs:

* `aircraft::Aircraft` : system to be simulated
* `steprange::AbstractArray` : defines each step of the simulation

Outputs:

* `omegas::Vector{Float64}` : a vector of rotational speeds in rad/s at the current step
* `Js::Array{Float64,2}` : each [i,j]th element is the advance ratio of the ith rotor at the jth step
* `Ts::Array{Float64,2}` : each [i,j]th element is the thrust of the ith rotor at the jth step
* `Qs::Array{Float64,2}` : each [i,j]th element is the torque of the ith rotor at the jth step
* `CTs::Array{Float64,2}` : each [i,j]th element is the thrust coefficient of the ith rotor at the jth step
* `CQs::Array{Float64,2}` : each [i,j]th element is the torque coefficient of the ith rotor at the jth step
* `ηs::Array{Float64,2}` : each [i,j]th element is the propulsive efficiency of the ith rotor at the jth step
* `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
* `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section
* `wakefunction::Function` : function accepts position X and returns the rotor induced velocity
* `wakeshapefunctions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
* `axialinterpolation::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `swirlinterpolation::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
* `axialmultiplier::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
* `swirlmultiplier::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor
* `CLs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing lift coefficients at each step
* `CDs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing drag coefficients at each step
* `CYs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing side force coefficients at each step
* `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
* `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
* `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
* `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface

"""
function solve_vlm_bem(aircraft, steprange)
    params_solve_rotor_nondimensional = solve_rotor_nondimensional(aircraft, steprange) # omegas, Js, Ts, Qs, us, vs
    params_solve_rotor_wake = solve_rotor_wake(aircraft, steprange) # wakefunctions, us, vs
    params_solve_wing_CF = solve_wing_CF(aircraft, steprange) # CLs, CDs, CYs
    params_lift_moment_distribution = lift_moment_distribution(aircraft, steprange) # cls, cds, cys, cmxs, cmys, cmzs

    return params_solve_rotor_nondimensional..., params_solve_rotor_wake[[1,4,5,6,7,8]]..., params_solve_wing_CF[2:end]..., params_lift_moment_distribution...
end
