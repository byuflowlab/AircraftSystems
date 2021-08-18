#=##############################################################################################
Filename: lift_distribution_template.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to calculate the lift
        distribution of a single wing over the specified alphas.
=###############################################################################################


# initialize parameters
"""
    LiftDistribution <: Parameters

# Fields

* `CFs::Array{Float64,2}` : [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
* `CMs::Array{Float64,2}` : [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth step
* `cfs::Vector{Vector{Array{TF,2}}}`
* `cms::Vector{Vector{Array{TF,2}}}`
* `wakefunctions::V4`: additional velocity function; default is nothing
* `plot_directory::String`
* `plot_base_name::String`
* `plot_extension::String`
* `plotstepi::AbstractRange`
* `surfacenames::Vector{String}`

"""
struct LiftDistribution{TF,TR} <: Parameters
    CFs::Array{TF,2}
    CMs::Array{TF,2}
    cfs::Vector{Vector{Array{TF,2}}}
    cms::Vector{Vector{Array{TF,2}}}
    wakefunctions::Vector{Function}
    plot_directory::String
    plot_base_name::String
    plot_extension::String
    plotstepi::TR
    surfacenames::Vector{String}
end

LiftDistribution(CFs, CMs, cfs, cms, plot_directory, plot_base_name, plot_extension, ploti, surfacenames; wakefunctions=[nothing]) =
    LiftDistribution(CFs, CMs, cfs, cms, wakefunctions, plot_directory, plot_base_name, plot_extension, ploti, surfacenames)

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(alphas, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(alphas, omega, nblades...)


"""
    lift_distribution_template(ploti, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip;
        plot_directory = joinpath(topdirectory, "data","plots",TODAY),
        plot_base_name = "default",
        plot_extension = ".pdf",
        step_symbol = L"\alpha ",
        surfacenames = ["default wing"],
        kwargs...)

# Arguments:

* `vinfs::Vector{Float64}`: freestream velocity for each time step
* `ploti`
* `alphas::Vector{Float64}`
* `wing_b::Float64`
* `wing_TR::Float64`
* `wing_AR::Float64`
* `wing_θroot::Float64`
* `wing_θtip::Float64`

# Keyword Arguments:

* `plot_directory = joinpath(topdirectory, "data","plots")`: directory to which plots are saved
* `plot_base_name = "default"`: first part of saved plot filenames
* `plot_extension = ".pdf"`: saved plots file extension
* `step_symbol`
* `surfacenames`

"""
function lift_distribution_template(ploti, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ;
            plot_directory=joinpath(topdirectory,"data","plots",TODAY),
            plot_base_name="default",
            plot_extension=".pdf",
            step_symbol=L"\alpha ",
            surfacenames=["default wing"],
            kwargs...)

    # prepare subsystems
    wings = simplewingsystem(wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, wing_le_sweep, wing_ϕ; kwargs...)
    rotors = CCBladeSystem(
        Vector{CC.Rotor}(undef,0),
        Vector{Vector{CC.Section{Float64, Float64, Float64, nothing}}}(undef,0),
        Vector{Vector{Float64}}(undef,0),
        Vector{Int64}(undef,0),
        Vector{Vector{Float64}}(undef,0),
        Vector{Vector{Float64}}(undef,0),
        Vector{Bool}(undef,0)
    )
    inertia = nothing
    nonliftingbodies = nothing
    structures = nothing
    motors = nothing
    batteries = nothing

    # build system struct
    aircraft = Aircraft(wings, rotors, inertia, nonliftingbodies, structures, motors, batteries)

    # compile actions
    actions = [solve_wing_CF_CM, lift_moment_distribution]

    # initialize parameters
    wakefunctions, CFs, CMs = solve_wing_CF_CM(aircraft, alphas) # wakefunctions, CFs, CMs
    cfs, cms = lift_moment_distribution(aircraft, alphas) # let step_range be replaced by alphas
    _, _, _, _, _, _, _ = post_plot_lift_moment_distribution(aircraft, alphas)

    # check sizes and instantiate struct
    @assert size(CFs) == size(CMs) "size of CFs and CMs inconsistent"
    @assert size(CFs)[2] == length(alphas) "length of parameter CFs and alphas inconsistent"
    @assert size(CFs)[1] == 3 "first dimension of CFs should be of length 1"
    nspanwisepanels1 = size(aircraft.wingsystem.system.surfaces[1])[2]
    @assert length(cfs[1]) == length(surfacenames) "length of cfs is inconsistent: expected $(length(surfacenames)); got $(length(cfs))"
    @assert length(cms[1]) == length(surfacenames) "length of cms is inconsistent: expected $(length(surfacenames)); got $(length(cms))"
    @assert size(cfs[1][1]) == (3, nspanwisepanels1) "size of cfs[1] is inconsistent: expected $((3, nspanwisepanels1)); got $(size(cfs[1]))"

    # prepare plot directory
    if !isdir(plot_directory); mkpath(plot_directory); end

    # build parameters struct
    parameters = LiftDistribution(CFs, CMs, cfs, cms, wakefunctions, plot_directory, plot_base_name, plot_extension, ploti, surfacenames)

    # build freestream_function
    function freestream_function(aircraft, parameters, environment, alphas, stepi)

        # calculate freestream
        vinf = 1.0 # + ti # arbitrary for lift distribution?
        alpha = alphas[stepi]
        beta = 0.0
        Omega = StaticArrays.@SVector zeros(3)
        freestream = Freestream(vinf, alpha, beta, Omega)

        return freestream
    end

    # build environment_function
    function environment_function(aircraft, parameters, alphas, stepi)
        Environment() # arbitrary
    end

    # compile postactions
    postactions = [post_plot_cf_cm_alpha_sweep, post_plot_lift_moment_distribution]

    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, alphas) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, alphas, step_symbol
end
