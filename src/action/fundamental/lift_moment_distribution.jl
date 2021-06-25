#=##############################################################################################
Filename: lift_moment_distribution.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to extract the lifting line lift and moment distributions
=###############################################################################################
using StaticArrays
using LinearAlgebra

"""
    lift_moment_distribution(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol) <: Action

Solves for the aerodynamic force distribution at each step.

NOTE: THIS ACTION DOES NOT SOLVE THE VORTEX LATTICE. Call `solve_CF` prior to calling this action.

# Arguments:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `steprange::AbstractArray` : array of steps for which the simulation is run
* `stepi::Int` : index of the current step
* `stepsymbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `cfs::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl
* `cms::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz
* `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
* `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
* `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
* `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface

"""
function lift_moment_distribution(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    
    # extract lift and moment distribution
    cfs, cms = VL.lifting_line_coefficients(aircraft.wingsystem.system, aircraft.wingsystem.lifting_line_rs, aircraft.wingsystem.lifting_line_chords)

    # store to `parameters`
    nwings = length(aircraft.wingsystem.system.surfaces)
    for iwing in 1:nwings
        parameters.cfs[iwing] = cfs[iwing]
        parameters.cms[iwing] = cms[iwing]
        parameters.cls[iwing][:,stepi] = cfs[iwing][3,:]
        parameters.cds[iwing][:,stepi] = cfs[iwing][1,:]
        parameters.cys[iwing][:,stepi] = cfs[iwing][2,:]
        parameters.cmxs[iwing][:,stepi] = cms[iwing][1,:]
        parameters.cmys[iwing][:,stepi] = cms[iwing][2,:]
        parameters.cmzs[iwing][:,stepi] = cms[iwing][3,:]
    end

    return false
end


"""
    lift_moment_distribution_blownwing(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol) <: Action

Solves for the aerodynamic force distribution at each step. This method solves for the lifting line coefficients without using rotor induced velocity in a blown wing setup, as used in Epema (2017).

NOTE: THIS ACTION DOES NOT SOLVE THE VORTEX LATTICE. Call `solve_CF` prior to calling this action.

# Arguments:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `steprange::AbstractArray` : array of steps for which the simulation is run
* `stepi::Int` : index of the current step
* `stepsymbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `cfs::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl
* `cms::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz
* `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
* `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
* `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
* `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface

"""
function lift_moment_distribution_blownwing(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
    
    # extract lift and moment distribution
    cfs, cms = lifting_line_coefficients_blownwing(aircraft.wingsystem.system, aircraft.wingsystem.lifting_line_rs, aircraft.wingsystem.lifting_line_chords, parameters.wakefunctions[stepi])

    # store to `parameters`
    nwings = length(aircraft.wingsystem.system.surfaces)
    for iwing in 1:nwings
        parameters.cfs[iwing] = cfs[iwing]
        parameters.cms[iwing] = cms[iwing]
        parameters.cls[iwing][:,stepi] = cfs[iwing][3,:]
        parameters.cds[iwing][:,stepi] = cfs[iwing][1,:]
        parameters.cys[iwing][:,stepi] = cfs[iwing][2,:]
        parameters.cmxs[iwing][:,stepi] = cms[iwing][1,:]
        parameters.cmys[iwing][:,stepi] = cms[iwing][2,:]
        parameters.cmzs[iwing][:,stepi] = cms[iwing][3,:]
    end

    return false
end

"""
    lift_moment_distribution(aircraft, steprange)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : system to be simulated
* `steprange::AbstractArray` : defines each step of the simulation

# Returns:

* `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
* `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
* `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
* `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
* `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface

"""
function lift_moment_distribution(aircraft, steprange)

    nwings = length(aircraft.wingsystem.system.surfaces)
    nspanwisepanels = [size(surface)[2] for surface in aircraft.wingsystem.system.surfaces]
    cls = [zeros(nspanwisepanels[i], length(steprange)) for i in 1:nwings]
    cds = deepcopy(cls)
    cys = deepcopy(cls)
    cmxs = deepcopy(cls)
    cmys = deepcopy(cls)
    cmzs = deepcopy(cls)

    return cls, cds, cys, cmxs, cmys, cmzs
end


"""
    lifting_line_coefficients_blownwing(system, r, c, wakefunction)

This version solves for the lifting line coefficients without the rotor induced velocities in the equation. For simplicity this neglects the left and right vortex filaments.

Modeled after `lifting_line_coefficients` in `VortexLattice.jl`.
"""
function lifting_line_coefficients_blownwing(system, r, c, wakefunction)
    TF = promote_type(eltype(system), eltype(eltype(r)), eltype(eltype(c)))
    nsurf = length(system.surfaces)
    cf = Vector{Matrix{TF}}(undef, nsurf)
    cm = Vector{Matrix{TF}}(undef, nsurf)
    for isurf = 1:nsurf
        ns = size(system.surfaces[isurf], 2)
        cf[isurf] = Matrix{TF}(undef, 3, ns)
        cm[isurf] = Matrix{TF}(undef, 3, ns)
    end
    return lifting_line_coefficients_blownwing!(cf, cm, system, r, c, wakefunction)
end


"""
    lifting_line_coefficients_blownwing!(cf, cm, system, r, c)

For simplicity, this neglects effects of the left and right vortex filaments.

Modeled after `lifting_line_coefficients!` in `VortexLattice.jl`.
"""
function lifting_line_coefficients_blownwing!(cf, cm, system, r, c, wakefunction)

    # number of surfaces
    nsurf = length(system.surfaces)

    # check that a near field analysis has been performed
    @assert system.near_field_analysis[] "Near field analysis required"

    # extract reference properties
    ref = system.reference[]

    # iterate through each lifting surface
    for isurf = 1:nsurf

        nc, ns = size(system.surfaces[isurf])

        # extract current surface panels and panel properties
        panels = system.surfaces[isurf]
        properties = system.properties[isurf]
        cr = CartesianIndices(panels)

        # loop through each chordwise set of panels
        for j = 1:ns

            I = cr[j]

            # calculate segment length
            rls = SVector(r[isurf][1,j], r[isurf][2,j], r[isurf][3,j])
            rrs = SVector(r[isurf][1,j+1], r[isurf][2,j+1], r[isurf][3,j+1])
            ds = norm(rrs - rls)

            # calculate reference location
            rs = (rls + rrs)/2

            # calculate reference chord
            cs = (c[isurf][j] + c[isurf][j+1])/2

            # calculate section force and moment coefficients
            cf[isurf][:,j] .= 0.0
            cm[isurf][:,j] .= 0.0
            for i = 1:nc
                # add influence of bound vortex
                rb = VL.top_center(panels[i,j])
                Γ = properties[i,j].gamma
                Vlocal = properties[i,j].velocity 
                Vlocal -= wakefunction(rb)

                Δs = VL.top_vector(panels[I]) 
                cfb = VL.RHO * Γ * cross(Vlocal, Δs)

                # cfb = properties[i,j].cfb
                cf[isurf][:,j] .+= cfb
                cm[isurf][:,j] .+= cross(rb - rs, cfb)
            end

            # update normalization
            # cf[isurf][:,j] .*= 2 ./ (cs * (VL.freestream_velocity(system.freestream[1])) .^ 2)
            cf[isurf][:,j] .*= ref.S/(ds*cs)
            cm[isurf][:,j] .*= ref.S/(ds*cs^2)
        end
    end

    return cf, cm
end