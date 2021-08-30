#=##############################################################################################
Filename: lift_moment_distribution.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to extract the lifting line lift and moment distributions
=###############################################################################################
using StaticArrays
using LinearAlgebra

"""
    lift_moment_distribution(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol) <: Action

Solves for the aerodynamic force distribution at each step.

NOTE: THIS ACTION DOES NOT SOLVE THE VORTEX LATTICE. Call `solve_wing_CF_CM` prior to calling this action.

# Arguments:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `step_range::AbstractArray` : array of steps for which the simulation is run
* `stepi::Int` : index of the current step
* `step_symbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `cfs::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl corresponding to the ith step, jth lifting surface
* `cms::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz corresponding to the ith step, jth lifting surface

"""
function lift_moment_distribution(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    # extract lift and moment distribution
    cfs, cms = VL.lifting_line_coefficients(aircraft.wing_system.system, aircraft.wing_system.lifting_line_rs, aircraft.wing_system.lifting_line_chords; frame = VL.Wind())
    # store to `parameters`
    nwings = length(aircraft.wing_system.system.surfaces)
    parameters.cfs[stepi] .= cfs
    parameters.cms[stepi] .= cms
    # for iwing in 1:nwings

    # end

    return false
end


"""
    lift_moment_distribution_blownwing(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol) <: Action

Solves for the aerodynamic force distribution at each step. This method solves for the lifting line coefficients without using rotor induced velocity in a blown wing setup, as used in Epema (2017).

NOTE: THIS ACTION DOES NOT SOLVE THE VORTEX LATTICE. Call `solve_CF` prior to calling this action.

# Arguments:

* `aircraft::Aircraft` : `Aircraft` system object
* `parameters<:Parameters` `Parameters` struct
* `freestream::Freestream` : `Freestream` object
* `environment::Environment` `Environment` object
* `step_range::AbstractArray` : array of steps for which the simulation is run
* `stepi::Int` : index of the current step
* `step_symbol::String` : defines the step, e.g. `alpha` or `time`

`parameters <: Parameters` requires the following elements:

* `cfs::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl corresponding to the ith step, jth lifting surface
* `cms::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz corresponding to the ith step, jth lifting surface

"""
function lift_moment_distribution_blownwing(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)

    # extract lift and moment distribution
    cfs, cms = VL.lifting_line_coefficients(aircraft.wing_system.system, aircraft.wing_system.lifting_line_rs, aircraft.wing_system.lifting_line_chords; frame = VL.Wind())

    # store to `parameters`
    parameters.cfs[stepi] .= cfs
    parameters.cms[stepi] .= cms

    # correct lift coefficients as done by Epema
    nwings = length(aircraft.wing_system.system.surfaces)
    Vref = aircraft.wing_system.system.reference[1].V
    Vinf = aircraft.wing_system.system.freestream[1].Vinf
    for iwing in 1:nwings
        # first determine cl w/o rotor induced velocities
        panels = aircraft.wing_system.system.surfaces[iwing]
        n_spanwise_sections = size(aircraft.wing_system.system.properties[iwing])[2]
        gammas = [Vref * sum([panel.gamma for panel in aircraft.wing_system.system.properties[iwing][:,i_spanwise_section]])  for i_spanwise_section in 1:n_spanwise_sections]
        velocities = [panel.velocity * Vref for panel in aircraft.wing_system.system.properties[iwing]]
        vwakes = parameters.wake_function[stepi].(VL.top_center.(panels))
        # vwakes_minus_y = [[v[1]; 0.0; v[3]] for v in vwakes]
        # velocities_minus_rotor = velocities .- vwakes_minus_y #w/o rotor-induced velocities
        velocities_minus_rotor = velocities .- vwakes #w/o rotor-induced velocities
        Δs = VL.top_vector.(panels)
        v_perps = LA.cross.(velocities_minus_rotor, Δs) ./ LA.norm.(Δs)
        v_perp_norm = LA.norm.([[v[1]; 0.0; v[3]] for v in v_perps])
        # TODO: interpote to get 1/4 chord velocity
        v_perp = [Statistics.mean(v_perp_norm[:,i]) for i in 1:n_spanwise_sections]
        # this step normalizes by the mac
        cls = 2 * gammas .* v_perp ./ (aircraft.wing_system.cmacs[iwing] * Vinf^2)
        # store to parameters
        parameters.cfs[stepi][iwing][3,:] .= cls
    end

    return false
end

"""
    lift_moment_distribution(aircraft, step_range)

Method returns initialized elements required for the `parameters <: Parameters` struct during simulation.

# Arguments:

* `aircraft::Aircraft` : system to be simulated
* `step_range::AbstractArray` : defines each step of the simulation

# Returns:

* `cfs::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl corresponding to the ith step, jth lifting surface
* `cms::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz corresponding to the ith step, jth lifting surface

"""
function lift_moment_distribution(aircraft, step_range)

    nwings = length(aircraft.wing_system.system.surfaces)
    nspanwisepanels = [size(surface)[2] for surface in aircraft.wing_system.system.surfaces]
    cfs = [[zeros(3, length(aircraft.wing_system.system.surfaces[i])) for i in 1:nwings] for j in 1:length(step_range)]
    cms = deepcopy(cfs)

    return cfs, cms
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
