#=##############################################################################################
Filename: lift_distribution_template.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to calculate the lift
        distribution of a single wing over the specified alphas.
=###############################################################################################


# initialize parameters
"""
    LiftDistribution{V1,V2,V3,V4,V5,V6,V7} <: Parameters

# Fields

* `CLs::V1`
* `CDs::V1`
* `CYs::V1`
* `cls::V2`
* `cds::V2`
* `cys::V2`
* `cmxs::V2`
* `cmys::V2`
* `cmzs::V2`
* `cfs::V3`
* `cms::V3`
* `wakefunctions::V4`: additional velocity function; default is nothing
* `plotdirectory::V5`
* `plotbasename::V5`
* `plotextension::V5`
* `plotstepi::V6`
* `surfacenames::V7`

"""
struct LiftDistribution{V1,V2,V3,V4,V5,V6,V7} <: Parameters
    CLs::V1
    CDs::V1
    CYs::V1
    cls::V2
    cds::V2
    cys::V2
    cmxs::V2
    cmys::V2
    cmzs::V2
    cfs::V3
    cms::V3
    wakefunctions::V4
    plotdirectory::V5
    plotbasename::V5
    plotextension::V5
    plotstepi::V6
    surfacenames::V7
end

LiftDistribution(CLs, CDs, CYs, cls, cds, cys, cmxs, cmys, cmzs, cfs, cms, plotdirectory, plotbasename, plotextension, ploti, surfacenames; wakefunctions=[nothing]) = 
    LiftDistribution(CLs, CDs, CYs, cls, cds, cys, cmxs, cmys, cmzs, cfs, cms, wakefunctions, plotdirectory, plotbasename, plotextension, ploti, surfacenames)

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(alphas, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(alphas, omega, nblades...)


"""
    lift_distribution_template(ploti, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip;
        plotdirectory = joinpath(topdirectory, "data","plots",TODAY),
        plotbasename = "default",
        plotextension = ".pdf",
        stepsymbol = L"\alpha ",
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

* `plotdirectory = joinpath(topdirectory, "data","plots")`: directory to which plots are saved
* `plotbasename = "default"`: first part of saved plot filenames
* `plotextension = ".pdf"`: saved plots file extension
* `stepsymbol`
* `surfacenames`

"""
function lift_distribution_template(ploti, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip;
            plotdirectory=joinpath(topdirectory,"data","plots",TODAY),
            plotbasename="default",
            plotextension=".pdf",
            stepsymbol=L"\alpha ",
            surfacenames=["default wing"],
            kwargs...)

    # prepare subsystems
<<<<<<< HEAD
    wings = simplewingsystem(wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip; kwargs...)
=======
    wings = simplewingsystem(; wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, kwargs...)
>>>>>>> 779f9ec261fa9820596a550792d2e421c84812ea
    rotors = nothing
    nonliftingbodies = nothing
    structures = nothing
    motors = nothing
    batteries = nothing

    # build system struct
    aircraft = Aircraft(wings, rotors, nonliftingbodies, structures, motors, batteries)

    # compile actions
    actions = [solve_wing_CF, lift_moment_distribution]
    
    # initialize parameters
    wakefunctions, CLs, CDs, CYs = solve_wing_CF(aircraft, alphas) # let steprange be replaced by alphas
    cls, cds, cys, cmxs, cmys, cmzs = lift_moment_distribution(aircraft, alphas) # let steprange be replaced by alphas
    _, _, _, _, _, _, cfs, cms, _, _, _, _ = post_plot_lift_moment_distribution(aircraft, alphas)

    # check sizes and instantiate struct
    @assert length(CLs) == length(alphas) "length of parameter CLs and alphas inconsistent"
    @assert length(CDs) == length(alphas) "length of parameter CDs and alphas inconsistent"
    @assert length(CYs) == length(alphas) "length of parameter CYs and alphas inconsistent"
    nspanwisepanels1 = size(aircraft.wingsystem.system.surfaces[1])[2]
    @assert size(cls[1]) == (nspanwisepanels1, length(alphas)) "size of cls is inconsistent with simulation data for the first surface: expected $((nspanwisepanels1, length(alphas))); got $(size(cls[1]))"
    @assert size(cds[1]) == (nspanwisepanels1, length(alphas)) "size of cds is inconsistent with simulation data for the first surface: expected $((nspanwisepanels1, length(alphas))); got $(size(cds[1]))"
    @assert size(cys[1]) == (nspanwisepanels1, length(alphas)) "size of cys is inconsistent with simulation data for the first surface: expected $((nspanwisepanels1, length(alphas))); got $(size(cys[1]))"
    @assert size(cmxs[1]) == (nspanwisepanels1, length(alphas)) "size of cmxs is inconsistent with simulation data for the first surface: expected $((nspanwisepanels1, length(alphas))); got $(size(cmxs[1]))"
    @assert size(cmys[1]) == (nspanwisepanels1, length(alphas)) "size of cmys is inconsistent with simulation data for the first surface: expected $((nspanwisepanels1, length(alphas))); got $(size(cmys[1]))"
    @assert size(cmzs[1]) == (nspanwisepanels1, length(alphas)) "size of cmzs is inconsistent with simulation data for the first surface: expected $((nspanwisepanels1, length(alphas))); got $(size(cmzs[1]))"
    @assert length(cfs) == length(surfacenames) "length of cfs is inconsistent: expected $(length(surfacenames)); got $(length(cfs))"
    @assert length(cms) == length(surfacenames) "length of cms is inconsistent: expected $(length(surfacenames)); got $(length(cms))"
    @assert size(cfs[1]) == (3, nspanwisepanels1) "size of cfs[1] is inconsistent: expected $((3, nspanwisepanels1)); got $(size(cfs[1]))"
    
    # prepare plot directory
    if !isdir(plotdirectory); mkpath(plotdirectory); end
    
    # build parameters struct
    parameters = LiftDistribution(CLs, CDs, CYs, cls, cds, cys, cmxs, cmys, cmzs, cfs, cms, wakefunctions, plotdirectory, plotbasename, plotextension, ploti, surfacenames)
    
    # build freestream_function
    function freestream_function(aircraft, parameters, environment, alphas, stepi)
        
        # calculate freestream
        vinf = 1.0 # + ti # arbitrary for lift distribution?
        alpha = alphas[stepi]
        beta = 0.0
        Omega = zeros(3)
        freestream = Freestream(vinf, alpha, beta, Omega)
        
        return freestream
    end
    
    # build environment_function
    function environment_function(aircraft, parameters, alphas, stepi)
        Environment() # arbitrary
    end
    
    # compile postactions
    postactions = [post_plot_cl_alpha_sweep, post_plot_lift_moment_distribution]
    
    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, alphas) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, alphas, stepsymbol
end
