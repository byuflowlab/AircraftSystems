# #=##############################################################################################
# Filename: couple_vlm_bem.jl
# Author: Ryan Anderson
# Contact: rymanderson@gmail.com
# README: `<: Action` function calculates aerodynamic forces on a blade-element momentum-vortex lattice system
# =###############################################################################################

# """
# couple_vlm_bem <: Action

# Solves for the aerodynamic force distribution at each step.

# NOTE: THIS ACTION DOES NOT SOLVE THE VORTEX LATTICE. Call `solve_wing_CF_CM` prior to calling this action.

# Inputs:

# * `aircraft::Aircraft` : `Aircraft` system object
# * `parameters<:Parameters` `Parameters` struct
# * `freestream::Freestream` : `Freestream` object
# * `environment::Environment` `Environment` object
# * `step_range::AbstractArray` : array of steps for which the simulation is run
# * `stepi::Int` : index of the current step
# * `step_symbol::String` : defines the step, e.g. `alpha` or `time`

# `parameters <: Parameters` requires the following elements:

# * `cfs::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl
# * `cms::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz
# * `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
# * `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
# * `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
# * `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
# * `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
# * `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface
# * `omegas::Vector{Float64}` : a vector of length `length(step_range)` containing a vector of rotational velocities for each rotor
# * `Js::Vector{Vector{Float64}}` : a vector of length `length(step_range)` containing a vector of advance ratios for each rotor
# * `CTs::Vector{Vector{Float64}}` : a vector of length `length(step_range)` containing a vector of thrust coefficients for each rotor
# * `CQs::Vector{Vector{Float64}}` : a vector of length `length(step_range)` containing a vector of torque coefficients for each rotor
# * `??s::Vector{Vector{Float64}}` : a vector of length `length(step_range)` containing a vector of propulsive efficiencies for each rotor

# """
# function couple_vlm_bem(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)
#     # extract lift and moment distribution
#     cfs, cms = VL.lifting_line_coefficients(aircraft.wingsystem.system, aircraft.wingsystem.lifting_line_rs, aircraft.wingsystem.lifting_line_chords)
#     # store to `parameters`
#     nwings = length(aircraft.wingsystem.system.surfaces)
#     for iwing in 1:nwings
#         parameters.cls[iwing][:,stepi] = cfs[iwing][3,:]
#         parameters.cds[iwing][:,stepi] = cfs[iwing][1,:]
#         parameters.cys[iwing][:,stepi] = cfs[iwing][2,:]
#         parameters.cmxs[iwing][:,stepi] = cms[iwing][1,:]
#         parameters.cmys[iwing][:,stepi] = cms[iwing][2,:]
#         parameters.cmzs[iwing][:,stepi] = cms[iwing][3,:]
#     end
#     return false
# end

# """
# lift_moment_distribution(system, step_range)

# Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Inputs:

# * `aircraft::Aircraft` : system to be simulated
# * `step_range::AbstractArray` : defines each step of the simulation

# Outputs:

# * `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
# * `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
# * `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
# * `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
# * `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
# * `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface

# """
# function lift_moment_distribution(aircraft, step_range)

#     nwings = length(aircraft.wingsystem.system.surfaces)
#     nspanwisepanels = [size(surface)[2] for surface in aircraft.wingsystem.system.surfaces]
#     cls = [zeros(nspanwisepanels[i], length(step_range)) for i in 1:nwings]
#     cds = deepcopy(cls)
#     cys = deepcopy(cls)
#     cmxs = deepcopy(cls)
#     cmys = deepcopy(cls)
#     cmzs = deepcopy(cls)

#     return cls, cds, cys, cmxs, cmys, cmzs
# end
