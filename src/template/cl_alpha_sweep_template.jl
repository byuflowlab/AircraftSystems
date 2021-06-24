#=##############################################################################################
Filename: wing_cl_alpha_sweep.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to prepare a single rotor and
        sweep over advance ratios.
=###############################################################################################

# initialize parameters
"""
    CLAlphaSweep{V1,V2,V3,V4,V5,V6} <: Parameters

# Fields:

* `CLs::V1`
* `CDs::V2`
* `CYs::V3`
* `plotdirectory::V4`
* `plotbasename::V5`
* `plotextension::V6`

"""
struct CLAlphaSweep{V1,V2,V3,V4,V5,V6} <: Parameters
    CLs::V1
    CDs::V2
    CYs::V3
    plotdirectory::V4
    plotbasename::V5
    plotextension::V6
end

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(alphas, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(alphas, omega, nblades...)

"""
    cl_alpha_sweep_template(alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip;
        plotdirectory = joinpath(topdirectory, "data","plots",TODAY),
        plotbasename = "default",
        plotextension = ".pdf",
        stepsymbol = L"\alpha[^\circ]",
        kwargs...)

Arguments:
* `alphas`
* `wing_b`
* `wing_TR`
* `wing_AR`
* `wing_θroot`
* `wing_θtip`

Keyword Arguments:
* `plotdirectory`
* `plotbasename`
* `plotextension`
* `stepsymbol`

"""
function cl_alpha_sweep_template(alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip;
            plotdirectory=joinpath(topdirectory,"data","plots",TODAY),
            plotbasename="default",
            plotextension=".pdf",
            stepsymbol=L"\alpha[^\circ]",
            kwargs...)

    # prepare subsystems
    wings = simplewingsystem(; wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, kwargs...)
    rotors = nothing
    nonliftingbodies = nothing
    structures = nothing
    motors = nothing
    batteries = nothing

    # build system struct
    aircraft = Aircraft(wings, rotors, nonliftingbodies, structures, motors, batteries)

    # compile actions
    actions = [solve_CF]

    # initialize parameters
    CLs, CDs, CYs = solve_CF(aircraft, alphas) # let steprange be replaced by alphas

    # check sizes and instantiate struct
    @assert length(CLs) == length(alphas) "length of parameter CLs and alphas inconsistent"
    @assert length(CDs) == length(alphas) "length of parameter CDs and alphas inconsistent"
    @assert length(CYs) == length(alphas) "length of parameter CYs and alphas inconsistent"
    
    parameters = CLAlphaSweep(CLs, CDs, CYs, plotdirectory, plotbasename, plotextension)

    # build freestream_function
    function freestream_function(aircraft, parameters, environment, alphas, stepi)

        # calculate freestream
        Vinf = 1.0 # arbitrary for CL-alpha sweep
        alpha = alphas[stepi]
        beta = 0.0
        Omega = zeros(3)
        freestream = Freestream(Vinf, alpha, beta, Omega)

        return freestream
    end

    # build environment_function
    function environment_function(aircraft, parameters, alphas, stepi)
        Environment() # arbitrary
    end

    # compile postactions
    postactions = [post_plot_cl_alpha_sweep]

    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, alphas) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, alphas, stepsymbol
end
