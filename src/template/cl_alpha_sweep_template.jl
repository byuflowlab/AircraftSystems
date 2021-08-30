#=##############################################################################################
Filename: wing_cl_alpha_sweep.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to prepare a single rotor and
        sweep over advance ratios.
=###############################################################################################

# initialize parameters
"""
    CLAlphaSweep{TF,TS,FUN} <: Parameters

# Fields:

* `CFs::Array{Float64,2}` : [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
* `CMs::Array{Float64,2}` : [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth step
* `wake_function::Vector{FUN}`
* `plot_directory::TS`
* `plot_base_name::TS`
* `plot_extension::TS`

"""
struct CLAlphaSweep{TF,TS,FUN} <: Parameters
    CFs::Array{TF,2}
    CMs::Array{TF,2}
    wake_function::Vector{FUN}
    plot_directory::TS
    plot_base_name::TS
    plot_extension::TS
end

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(alphas, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(alphas, omega, nblades...)

"""
    cl_alpha_sweep_template(alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip;
        plot_directory = joinpath(topdirectory, "data","plots",TODAY),
        plot_base_name = "default",
        plot_extension = ".pdf",
        step_symbol = L"/alpha[^/circ]", # these forward slashes actually represent backslashes, but those cause an error in the docstrings
        kwargs...)

# Arguments:

* `alphas`
* `wing_b`
* `wing_TR`
* `wing_AR`
* `wing_θroot`
* `wing_θtip`

# Keyword Arguments:

* `plot_directory`
* `plot_base_name`
* `plot_extension`
* `step_symbol`

"""
function cl_alpha_sweep_template(alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ;
            plot_directory=joinpath(topdirectory,"data","plots",TODAY),
            plot_base_name="default",
            plot_extension=".pdf",
            step_symbol=L"\alpha[^\circ]",
            kwargs...)

    # prepare subsystems
    wings = simplewing_system(wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ; kwargs...)
    rotors = CCBladeSystem(
        Vector{CC.Rotor}(undef,0),
        Vector{Vector{CC.Section{Float64, Float64, Float64, nothing}}}(undef,0),
        Vector{Vector{Float64}}(undef,0),
        Vector{Int64}(undef,0),
        Vector{Vector{Float64}}(undef,0),
        Vector{Vector{Float64}}(undef,0),
        Vector{Bool}(undef,0)
    )

    # rotors::Vector{CC.Rotor}#{TF, Int64, Bool, T1, T2, T3, T4}}
    # sectionlists::Vector{Vector{CCBlade.Section{TF1, TF2, TF3, TAF}}}
    # rlists::Vector{Vector{TF}}
    # index::Vector{Int64}
    # positions::Vector{Vector{TF}}
    # orientations::Vector{Vector{TF}}
    # spin_directions::Vector{Bool}
    inertia = nothing
    nonliftingbodies = nothing
    structures = nothing
    motors = nothing
    batteries = nothing

    # build system struct
    aircraft = Aircraft(wings, rotors, inertia, nonliftingbodies, structures, motors, batteries)

    # compile actions
    actions = [solve_wing_CF_CM]

    # initialize parameters
    wake_function, CFs, CMs = solve_wing_CF_CM(aircraft, alphas) # wake_function, CFs, CMs

    # check sizes and instantiate struct
    @assert size(CFs) == size(CMs) "size of CFs and CMs inconsistent"
    @assert size(CFs)[2] == length(alphas) "length of parameter CFs and alphas inconsistent"
    @assert size(CFs)[1] == 3 "first dimension of CFs should be of length 1"

    parameters = CLAlphaSweep(CFs, CMs, wake_function, plot_directory, plot_base_name, plot_extension)

    # build freestream_function
    function freestream_function(aircraft, parameters, environment, alphas, stepi)

        # calculate freestream
        Vinf = 1.0 # arbitrary for CL-alpha sweep
        alpha = alphas[stepi]
        beta = 0.0
        Omega = StaticArrays.@SVector zeros(3)
        freestream = Freestream(Vinf, alpha, beta, Omega)

        return freestream
    end

    # build environment_function
    function environment_function(aircraft, parameters, alphas, stepi)
        Environment() # arbitrary
    end

    # compile postactions
    postactions = [post_plot_cf_cm_alpha_sweep]

    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, alphas) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, alphas, step_symbol
end
