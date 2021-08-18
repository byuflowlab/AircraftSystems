#=##############################################################################################
Filename: rotor_sweep.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to prepare a single rotor and
        sweep over advance ratios.
=###############################################################################################

# initialize parameters
"""
    RotorSweepParameters{V1,V2,V3,V4,V5} <: Parameters

# Fields:

* `omegas::V1`
* `Js::V2`
* `Ts::V2`
* `Qs::V2`
* `Ps::V2`
* `CTs::V2`
* `CQs::V2`
* `ηs::V2`
* `us::V3`
* `vs::V3`
* `rotor_names::V4`
* `plot_directory::V5`
* `plot_base_name::V5`
* `plot_extension::V5`

"""
struct RotorSweepParameters{TF} <: Parameters
    omegas::Array{TF,2}
    Js::Array{TF,2}
    Ts::Array{TF,2}
    Qs::Array{TF,2}
    Ps::Array{TF,2}
    CTs::Array{TF,2}
    CQs::Array{TF,2}
    ηs::Array{TF,2}
    us::Vector{Vector{Vector{TF}}}
    vs::Vector{Vector{Vector{TF}}}
    rotor_names::Union{Vector{String}, Vector{LaTeXString}}
    plot_directory::String
    plot_base_name::String
    plot_extension::String
end

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(Js, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(Js, omega, nblades...)

# build freestream_function
# function get_J(parameters::RotorSweepParameters{V1,V2,V3,V4,V5,V6}, ri, stepi) where {V1, V2 <:AbstractArray{<:Any,1}, V3, V4, V5, V6}
#     parameters.Js[stepi]
# end

# function get_J(parameters::RotorSweepParameters{V1,V2,V3,V4}, ri, stepi) where {V1, V2 <:AbstractArray{<:Any,2}, V3, V4, V5}
function get_J(parameters, ri, stepi)
    parameters.Js[ri,stepi]
end

"""
    rotor_sweep_template(Js, omegas, rotors;
        rotor_names = ["rotor 1"],
        plot_directory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        plot_base_name = "default",
        plot_extension = ".pdf",
        step_symbol = L"J",
        kwargs...
    )

# Arguments:

* `Js::Array{Float64,2}` : each [i,j]th element contains the desired advance ratio of the ith rotor at the jth step
* `omegas::Array{Float64,2}` : each [i,j]th element contains the commanded rotational velocity in rad/s of the ith rotor at the jth step
* `rotors::CCBladeSystem` : `CCBladeSystem` object defines the rotors.

# Keyword Arguments:

* `Res_list`
* `rotor_names`
* `plot_directory`
* `plot_base_name`
* `plot_extension`
* `step_symbol`

"""
function rotor_sweep_template(Js, omegas, rotors;
    rotor_names = ["rotor 1"],
    plot_directory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
    plot_base_name = "default",
    plot_extension = ".pdf",
    step_symbol = L"J",
    kwargs...
)

    # prepare subsystems
    wings = nothing
    inertia = nothing
    nonliftingbodies = nothing
    structures = nothing
    motors = nothing
    batteries = nothing

    # build system struct
    aircraft = Aircraft(wings, rotors, inertia, nonliftingbodies, structures, motors, batteries)

    # compile actions
    actions = [solve_rotor_nondimensional]

    # initialize parameters
    step_range = 1:size(Js)[2] # use time to define each part of the sweep
    # omegas_zero, Js_zero, CTs, CQs, ηs = solve_rotor(aircraft, step_range)
    omegas_zero, Js_zero, Ts, Qs, Ps, CTs, CQs, ηs, us, vs = solve_rotor_nondimensional(aircraft, step_range)

    # check data
    @assert typeof(Js) <: AbstractArray{<:Any,2} "Js must be a 2-dimensional array; got $(typeof(Js))"
    @assert size(omegas_zero) == size(omegas) "`omegas` has improper dimensions. \n\tExpected: $(size(omegas_zero))\n\tGot: $(size(omegas))"
    @assert size(Js_zero) == size(Js) "`Js` has improper dimensions. \n\tExpected: $(size(Js_zero))\n\tGot: $(size(Js))"
    # @warn isdir(plot_directory) "plot_directory does not exist at $plot_directory\n\ttry: `mkdir $plot_directory`"

    if !isdir(plot_directory); mkpath(plot_directory); @warn "plot directory $plot_directory does not exist; creating..."; end

    # build parameters
    parameters = RotorSweepParameters(omegas, Js, Ts, Qs, Ps, CTs, CQs, ηs, us, vs, rotor_names, plot_directory, plot_base_name, plot_extension)

    function freestream_function(aircraft, parameters, environment, step_range, stepi)

        J = get_J(parameters, 1, stepi)
        omega = parameters.omegas[1] # get omega for the first rotor
        n = omega / 2 / pi
        D = aircraft.rotorsystem.rotors[1].Rtip * 2

        # calculate freestream
        Vinf = J * n * D
        alpha = 0.0
        beta = 0.0
        Omega = StaticArrays.@SVector zeros(3)
        freestream = Freestream(Vinf, alpha, beta, Omega)

        return freestream
    end

    # build environment_function
    function environment_function(aircraft, parameters, step_range, stepi)
        Environment()
    end

    # assemble postactions
    postactions = [post_plot_rotor_sweep]

    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, step_range) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, step_range, step_symbol
end

"""
    rotor_sweep_template(Js, omegas,
        nblades_list, rhub_list, rtip_list, mach_correction,
        radii_list, chords_list, twists_list, ccblade_af_objects_list,
        index, positions, orientations, spin_directions;
            kwargs...
    )

Convenience function to create rotors with pre-created polars.

# Arguments

* `nblades_list::Vector{Int}` : number of blades of the ith rotor
* `rhub_list::Vector{Float64}` : hub radius of the ith rotor
* `rtip_list::Vector{Float64}` : tip radius of the ith rotor
* `mach_correction::Union{Nothing, CCBlade.MachCorrection}` : mach correction method used
* `radii_list::Vector{Vector{Float64}}` : ith element is a vector of radial stations defining the ith rotor
* `chords_list::Vector{Vector{Float64}}` : ith element is a vector of chords defining the ith rotor
* `twists_list::Vector{Vector{Float64}}` : ith element is a vector of twist angles in radians defining the ith rotor
* `ccblade_af_objects_list::Vector{Vector{String}}` : ith element is a vector of airfoil polar objects corresponding to the ith rotor
* `index::Vector{Int64}` : ith element is the index of `ccbladesystem.rotors` (the returned `CCBladeSystem` object) corresponding to the ith rotor of the system
* `positions::Vector{Vector{Float64}}` : ith element is a vector describing the hub position of the ith rotor in the body frame
* `orientations::Vector{Vector{Float64}}` : ith element is a vector describing the hub orientation (positive thrust direction) of the ith rotor in the body frame (back-right-up)
* `spin_directions::Vector{Bool}` : ith element is `true` if the ith rotor rotates in the same direction as the positive thrust direction by the right hand rule

"""
function rotor_sweep_template(Js, omegas, nblades_list, rhub_list, rtip_list, mach_correction, radii_list, chords_list, twists_list, ccblade_af_objects_list, index, positions, orientations, spin_directions; kwargs...)

    # prepare subsystems
    rotors = CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction, radii_list, chords_list, twists_list, ccblade_af_objects_list, index, positions, orientations, spin_directions; kwargs...)

    return rotor_sweep_template(Js, omegas, rotors; kwargs...)
end

"""
    rotor_sweep_template(Js, omegas,
        nblades_list, rhub_list, rtip_list, mach_correction,
        radii_list, chords_list, twists_list,

        index, positions, orientations, spin_directions;
            kwargs...
    )

Convenience function to create rotors and polars.

# Arguments

* `Res_lists::Vector{Vector{Vector{Int64}}}` : [i][j]th element is a vector of Reynold's numbers at which the ith rotor, jth section polar is evaluated
* `Ms_lists::Vector{Vector{Vector{Int64}}}` : [i][j]th element is a vector of Mach numbers at which the ith rotor, jth radial section polar is evaluated
* `contour_paths_list::Vector{Vector{String}}` : [i][j]th element is the path to the .dat airfoil contour file corresponding to the jth radial section of the ith rotor

"""
function rotor_sweep_template(Js, omegas, nblades_list, rhub_list, rtip_list, mach_correction, radii_list, chords_list, twists_list, Res_lists, Ms_lists, contour_paths_list, index, positions, orientations, spin_directions;
    polar_directory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
    kwargs...
)

    # prepare subsystems
    rotors = CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
        radii_list, chords_list, twists_list,
        Res_lists, Ms_lists, contour_paths_list,
        index, positions, orientations, spin_directions;
            polar_directory,
            kwargs...
    )

    return rotor_sweep_template(Js, omegas, rotors; kwargs...)
end
