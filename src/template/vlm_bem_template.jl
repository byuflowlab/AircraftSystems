#=##############################################################################################
Filename: blown_wing_template.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to calculate the lift
        distribution of a blown wing over the specified alphas.
=###############################################################################################

# initialize parameters
struct VLM_BEM{V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13,V14,V15} <: Parameters
    omegas::V1
    Js::V2
    Ts::V2
    Qs::V2
    CTs::V2
    CQs::V2
    ηs::V2
    us::V3
    vs::V3
    wakefunctions::V4
    wakeshapefunctions::V11
    axialinterpolations::V12
    swirlinterpolations::V13
    axialmultipliers::V14
    swirlmultipliers::V15
    CLs::V5
    CDs::V5
    CYs::V5
    cls::V6
    cds::V6
    cys::V6
    cmxs::V6
    cmys::V6
    cmzs::V6
    cfs::V7
    cms::V7
    surfacenames::V8
    rotornames::V8
    plotdirectory::V9
    plotbasename::V9
    plotextension::V9
    plotstepi::V10
end

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(alphas, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(alphas, omega, nblades...)

"""
    vlm_bem_template(vinfs, plotstepi, alphas,
        wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip,
        omegas, nblades, rhub, rtip, radii, chords, twists,
        airfoilcontours, airfoilnames, index,
        rotor_X, rotor_orientation, spindirections, Res_list, Ms_list;
            wakedevelopementfactor = 1.0, # fully developed by default
            swirlrecoveryfactor = 0.5, # as described in Veldhuis' paper
            surfacenames = ["default wing"],
            rotornames = ["rotor 1"],
            plotdirectory = joinpath(topdirectory, "data","plots",TODAY),
            plotbasename = "default",
            plotextension = ".pdf",
            stepsymbol = L"\alpha ",
            kwargs...
    ) <: Template

Template function returns inputs for a VLM + BEM simulation.

## Inputs

* `alphas::Vector{Float64}`
* `wing_b::Float64`
* `wing_TR::Float64`
* `wing_AR::Float64`
* `wing_θroot::Float64`
* `wing_θtip::Float64`

## Keyword Arguments

* `wakedevelopementfactor = 1.0` : value between 0-1 with 0 meaning not developed at all and 1 meaning fully developed
* `surfacenames = ["default wing"]` : names for each lifting surface used in plots
* `rotornames = ["rotor 1"]` : names for each rotor used in plots
* `plotdirectory = joinpath(topdirectory, "data","plots")` : directory to which plots are saved
* `plotbasename = "default"` : first part of saved plot filenames
* `plotextension = ".pdf"` : saved plots file extension
* `stepsymbol = L"\alpha "` : symbol in plots and terminal output describing each step

"""
function vlm_bem_template(vinfs, plotstepi, alphas,
    wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip,
    omegas, nblades, rhub, rtip, radii, chords, twists,
    airfoilcontours, airfoilnames, index,
    rotor_X, rotor_orientation, spindirections, Res_list, Ms_list;
        wakedevelopementfactor = 1.0, # fully developed by default
        swirlrecoveryfactor = 0.5, # as described in Veldhuis' paper
        surfacenames = ["default wing"],
        rotornames = ["rotor 1"],
        plotdirectory = joinpath(topdirectory, "data","plots",TODAY),
        plotbasename = "default",
        plotextension = ".pdf",
        stepsymbol = L"\alpha ",
        kwargs...
)
    # prepare subsystems
    wings = simplewingsystem(wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip; kwargs...)
    rotors = CCBladeSystem(nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, rotor_X, rotor_orientation, spindirections, Res_list, Ms_list; kwargs...)
    nonliftingbodies = nothing
    structures = nothing
    motors = nothing
    batteries = nothing

    # build system struct
    aircraft = Aircraft(wings, rotors, nonliftingbodies, structures, motors, batteries)

    # compile actions
    actions = [solve_vlm_bem]

    # initialize parameters
    steprange = alphas
    params_solve_vlm_bem = solve_vlm_bem(aircraft, steprange)
    params_solve_vlm_bem[1] .*= omegas

    # set wake development parameters
    @assert wakedevelopementfactor <= 1 && wakedevelopementfactor >= 0 "`wakedevelopementfactor` must be between 0 and 1"
    axialmultipliers = fill((distance2plane, Rtip) -> 1 + wakedevelopementfactor, length(aircraft.rotorsystem.index))
    swirlmultipliers = fill((distance2plane, Rtip) -> (1 + wakedevelopementfactor) * swirlrecoveryfactor, length(aircraft.rotorsystem.index))
    params_solve_vlm_bem = params_solve_vlm_bem[1:13]..., axialmultipliers, swirlmultipliers, params_solve_vlm_bem[16:end]...
    #= Contains:
        1 `omegas::Vector{Float64}` : a vector of rotational speeds in rad/s at the current step
        2 `Js::Array{Float64,2}` : each [i,j]th element is the advance ratio of the ith rotor at the jth step
        3 `Ts::Array{Float64,2}` : each [i,j]th element is the thrust of the ith rotor at the jth step
        4 `Qs::Array{Float64,2}` : each [i,j]th element is the torque of the ith rotor at the jth step
        5 `CTs::Array{Float64,2}` : each [i,j]th element is the thrust coefficient of the ith rotor at the jth step
        6 `CQs::Array{Float64,2}` : each [i,j]th element is the torque coefficient of the ith rotor at the jth step
        7 `ηs::Array{Float64,2}` : each [i,j]th element is the propulsive efficiency of the ith rotor at the jth step
        8 `us::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the axial induced velocity at ith step of the jth rotor at the kth radial section
        9 `vs::Vector{Vector{Vector{Float64}}}` : each [i][j][k]th element is the swirl induced velocity at ith step of the jth rotor at the kth radial section
        10 `wakefunction::Function` : function accepts position X and returns the rotor induced velocity
        11 `wakeshapefunctions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
        12 `axialinterpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
        13 `swirlinterpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
        14 `axialmultipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
        15 `swirlmultipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor
        16 `CLs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing lift coefficients at each step
        17 `CDs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing drag coefficients at each step
        18 `CYs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing side force coefficients at each step
        19 `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
        20 `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
        21 `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
        22 `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
        23 `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
        24 `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface
    =#

    params_post_plot_cl_alpha_sweep = post_plot_cl_alpha_sweep(aircraft, steprange)
    #= Contains:
        1 `CLs::Vector{Float64}` : CL corresponding to the ith step
        2 `CDs::Vector{Float64}` : CD corresponding to the ith step
        3 `CYs::Vector{Float64}` : CY corresponding to the ith step
        4 `plotdirectory::String` : directory where plots are saved
        5 `plotbasename::String` : first portion of the saved figure file name
        6 `plotextension::String` : extension of saved figure files
    =#

    params_post_plot_lift_moment_distribution = post_plot_lift_moment_distribution(aircraft, steprange)
    #= Contains:
        1 `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
        2 `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
        3 `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
        4 `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
        5 `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
        6 `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface
        7 `cfs::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl
        8 `cms::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy,
        9 `surfacenames::Vector{String}` : names of each lifting surface to be shown in the legend
        10 `plotdirectory::String` : path to the folder where plots will be saved
        11 `plotbasename::String` : first part of saved figure file names
        12 `plotextension::String` : extension of saved figure file names
        13 `plotstepi::Vector{Int}` : which steps at which to plot
    =#

    if !isnothing(surfacenames); params_post_plot_lift_moment_distribution[9] .= surfacenames; end

    params_post_plot_rotor_sweep = post_plot_rotor_sweep(aircraft, steprange)
    #= Contains:
        1 `Js::AbstractArray` : array of advance ratios of the ith rotor at the jth step
        2 `CTs::Array{Float64,2}` : array of thrust coefficients of the ith rotor at the jth step
        3 `CQs::Array{Float64,2}` : array of torque coefficients of the ith rotor at the jth step
        4 `ηs::Array{Float64,2}` : array of propulsive efficiencies of the ith rotor at the jth step
        5 `rotornames::Vector{String}` : vector of rotor names for use in plot legend
        6 `plotdirectory::String` : directory where plots are saved
        7 `plotbasename::String` : first portion of the saved figure file name
        8 `plotextension::String` : extension of saved figure files
    =#

    if !isnothing(rotornames); params_post_plot_rotor_sweep[5] .= rotornames; end
    # prepare plot directory
    if !isdir(plotdirectory); mkpath(plotdirectory); end
    println("==== MSG ====\n\tplotdirectory = $plotdirectory")
    # build parameters struct
    parameters = VLM_BEM(params_solve_vlm_bem..., params_post_plot_lift_moment_distribution[7:9]..., params_post_plot_rotor_sweep[5], plotdirectory, plotbasename, plotextension, plotstepi)
    # build freestream_function
    function freestream_function(aircraft, parameters, environment, alphas, stepi)
        # calculate freestream
        vinf = vinfs[stepi] # + ti # arbitrary for lift distribution?
        alpha = alphas[stepi]
        beta = 0.0
        Omega = zeros(3)
        freestream = Freestream(vinf, alpha, beta, Omega)
        # update reference
        # vwake = parameters.wakefunctions[stepi]
        reference = aircraft.wingsystem.system.reference[1]
        S = reference.S
        c = reference.c
        b = reference.b
        r = reference.r
        newV = freestream.vinf
        newreference = VL.Reference(S,c,b,r,newV)
        aircraft.wingsystem.system.reference[1] = newreference

        return freestream
    end

    # build environment_function
    function environment_function(aircraft, parameters, alphas, stepi)
        Environment() # arbitrary
    end

    # compile postactions
    postactions = [post_plot_cl_alpha_sweep, post_plot_lift_moment_distribution, post_plot_rotor_sweep]

    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, alphas) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, alphas, stepsymbol
end
