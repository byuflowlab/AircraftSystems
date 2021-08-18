#=##############################################################################################
Filename: blown_wing_template.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to calculate the lift
        distribution of a blown wing over the specified alphas.
=###############################################################################################

# initialize parameters
"""
    VLM_BEM <: Parameters

Parameters for a blown wing system.

# Fields:

* `omegas::Array{TF,2}`:
* `Js::Array{TF,2}`:
* `Ts::Array{TF,2}`:
* `Qs::Array{TF,2}`:
* `CTs::Array{TF,2}`:
* `CQs::Array{TF,2}`:
* `ηs::Array{TF,2}`:
* `us::Vector{Vector{Vector{TF}}}`:
* `vs::Vector{Vector{Vector{TF}}}`:
* `wakefunctions::Vector{Function}`:
* `wakeshapefunctions::Vector{Function}`:
* `axialinterpolations::Vector{Function}`:
* `swirlinterpolations::Vector{Function}`:
* `axialmultipliers::Vector{Function}`:
* `swirlmultipliers::Vector{Function}`:
* `CFs::Array{TF,2}`: [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
* `CMs::Array{TF,2}`: [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth step
* `cfs::Vector{Vector{Array{TF,2}}}`:
* `cms::Vector{Vector{Array{TF,2}}}`:
* `surfacenames::Vector{String}`:
* `rotor_names::Vector{String}`:
* `plot_directory::String`:
* `plot_base_name::String`:
* `plot_extension::String`:
* `plotstepi::TR`:

"""
struct VLM_BEM{TF,TR} <: Parameters
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
    wakefunctions::Vector{Function}
    wakeshapefunctions::Vector{Function}
    axialinterpolations::Vector{Function}
    swirlinterpolations::Vector{Function}
    axialmultipliers::Vector{Function}
    swirlmultipliers::Vector{Function}
    CFs::Array{TF,2}
    CMs::Array{TF,2}
    cfs::Vector{Vector{Array{TF,2}}}
    cms::Vector{Vector{Array{TF,2}}}
    surfacenames::Vector{String}
    rotor_names::Vector{String}
    plot_directory::String
    plot_base_name::String
    plot_extension::String
    plotstepi::TR
end

"""
    ParamsSolveVLMBEM{TF,TFUN} <: Parameters

Bare-bones `<:Parameters` struct for the `solve_vlm_bem` action.

"""
struct ParamsSolveVLMBEM{TF} <: Parameters
    omegas::Array{TF,2}
    Js::Array{TF,2}
    Ts::Array{TF,2}
    Qs::Array{TF,2}
    CTs::Array{TF,2}
    CQs::Array{TF,2}
    ηs::Array{TF,2}
    us::Vector{Vector{Vector{TF}}}
    vs::Vector{Vector{Vector{TF}}}
    wakefunctions::Vector{Function}
    wakeshapefunctions::Vector{Function}
    axialinterpolations::Vector{Function}
    swirlinterpolations::Vector{Function}
    axialmultipliers::Vector{Function}
    swirlmultipliers::Vector{Function}
    CFs::Array{TF,2}
    CMs::Array{TF,2}
    cfs::Vector{Array{TF,2}}
    cms::Vector{Array{TF,2}}
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
        rotor_positions, rotor_orientation, spin_directions, Res_list, Ms_list;
            wake_developement_factor = 1.0, # fully developed by default
            swirl_recovery_factor = 0.5, # as described in Veldhuis' paper
            surfacenames = ["default wing"],
            rotor_names = ["rotor 1"],
            plot_directory = joinpath(topdirectory, "data","plots",TODAY),
            plot_base_name = "default",
            plot_extension = ".pdf",
            step_symbol = L"\alpha ",
            kwargs...
    ) <: Template

Template function returns inputs for a VLM + BEM simulation.

# Arguments:

* `vinfs`
* `plotstepi`
* `alphas::Vector{Float64}`: angles of attack
* `wing_b::Float64`: wing span
* `wing_TR::Float64`: wing taper ratio
* `wing_AR::Float64`: wing aspect ratio
* `wing_θroot::Float64`: wing root twist
* `wing_θtip::Float64`: wing tip twist
* `omegas`
* `nblades::Int`
* `rhub`
* `rtip`
* `radii`
* `chords`
* `twists`
* `airfoilcontours`
* `airfoilnames`
* `index`
* `rotor_position`
* `rotor_orientation`
* `spin_directions`
* `Res_list`
* `Ms_list`

# Keyword Arguments:

* `wake_developement_factor` : value between 0-1 with 0 meaning not developed at all and 1 meaning fully developed
* `surfacenames` : names for each lifting surface used in plots
* `rotor_names` : names for each rotor used in plots
* `plot_directory` : directory to which plots are saved
* `plot_base_name` : first part of saved plot filenames
* `plot_extension` : saved plots file extension
* `step_symbol` : symbol in plots and terminal output describing each step

"""
function vlm_bem_template(vinfs, plotstepi, alphas, omegas,
    wingsystem::VortexLatticeSystem,
    rotorsystem::CCBladeSystem;
        wake_developement_factor = 1.0, # fully developed by default
        swirl_recovery_factor = 0.5, # as described in Veldhuis' paper
        surfacenames = ["default wing"],
        rotor_names = ["rotor 1"],
        mass = 0.0,
        inertia_x = 0.0,
        inertia_y = 0.0,
        inertia_z = 0.0,
        plot_directory = joinpath(topdirectory, "data","plots",TODAY),
        plot_base_name = "default",
        plot_extension = ".pdf",
        step_symbol = L"\alpha ",
        kwargs...
)
    # prepare subsystems
    wings = wingsystem
    rotors = rotorsystem
    inertia = Inertia(mass, inertia_x, inertia_y, inertia_z)
    nonliftingbodies = nothing
    structures = nothing
    motors = nothing
    batteries = nothing

    # build system struct
    aircraft = Aircraft(wings, rotors, inertia, nonliftingbodies, structures, motors, batteries)

    # compile actions
    actions = [solve_vlm_bem]

    # initialize parameters
    step_range = alphas
    params_solve_vlm_bem = solve_vlm_bem(aircraft, step_range)
    params_solve_vlm_bem[1] .*= omegas

    # set wake development parameters
    @assert wake_developement_factor <= 1 && wake_developement_factor >= 0 "`wake_developement_factor` must be between 0 and 1"
    axialmultipliers = Function[(distance2plane, Rtip) -> 2 for i in 1:length(aircraft.rotorsystem.index)]
    swirlmultipliers = Function[(distance2plane, Rtip) -> 1 for i in 1:length(aircraft.rotorsystem.index)]
    params_solve_vlm_bem = params_solve_vlm_bem[1:14]..., axialmultipliers, swirlmultipliers, params_solve_vlm_bem[17:end]...
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
        10 `wakefunctions::Vector{Function}` : function accepts position X and returns the rotor induced velocity
        11 `wakeshapefunctions::Vector{Function}` : [i]th element is a function f(Rtip, x) describing the radial distance from the rotor axis to the boundary of the wake of the ith rotor
        12 `axialinterpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the axial component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
        13 `swirlinterpolations::Vector{Function}` : [i]th element is a function f(rs, us, r, Rtip) that returns the swirl component of rotor-induced velocity at distance r from the rotor axis based on the calculated axial induced velocities output from CCBlade of the ith rotor
        14 `axialmultipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the axial induced velocity function of the ith rotor
        15 `swirlmultipliers::Vector{Function}` : [i]th element is a function f(distance2plane, Rtip) that is multiplied by the swirl induced velocity function of the ith rotor
        16 `CFs::Array{TF,2}`: [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
        17 `CMs::Array{TF,2}`: [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth
        18 `cfs::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl corresponding to the ith step, jth lifting surface
        19 `cms::Vector{Vector{Array{Float64,2}}}`: [i][j]th element is an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy, cmz corresponding to the ith step, jth lifting surface
    =#

    params_post_plot_cf_cm_alpha_sweep = post_plot_cf_cm_alpha_sweep(aircraft, step_range)
    #= Contains:
        1 `CFs::Array{TF,2}`: [:,j]th element is the [CD,CY,CL] force coefficients of the aircraft at the jth step
        2 `CMs::Array{TF,2}`: [:,j]th element is the [CMx,CMy,CMz] moment coefficients of the aircraft at the jth
        3 `plot_directory::String` : directory where plots are saved
        4 `plot_base_name::String` : first portion of the saved figure file name
        5 `plot_extension::String` : extension of saved figure files
    =#

    params_post_plot_lift_moment_distribution = post_plot_lift_moment_distribution(aircraft, step_range)
    #= Contains:
        1 `cfs::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl
        2 `cms::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy,
        3 `surfacenames::Vector{String}` : names of each lifting surface to be shown in the legend
        4 `plot_directory::String` : path to the folder where plots will be saved
        5 `plot_base_name::String` : first part of saved figure file names
        6 `plot_extension::String` : extension of saved figure file names
        7 `plotstepi::Vector{Int}` : which steps at which to plot
    =#

    if !isnothing(surfacenames); params_post_plot_lift_moment_distribution[3] .= surfacenames; end

    params_post_plot_rotor_sweep = post_plot_rotor_sweep(aircraft, step_range)
    #= Contains:
        1 `Js::AbstractArray` : array of advance ratios of the ith rotor at the jth step
        2 `CTs::Array{Float64,2}` : array of thrust coefficients of the ith rotor at the jth step
        3 `CQs::Array{Float64,2}` : array of torque coefficients of the ith rotor at the jth step
        4 `ηs::Array{Float64,2}` : array of propulsive efficiencies of the ith rotor at the jth step
        5 `rotor_names::Vector{String}` : vector of rotor names for use in plot legend
        6 `plot_directory::String` : directory where plots are saved
        7 `plot_base_name::String` : first portion of the saved figure file name
        8 `plot_extension::String` : extension of saved figure files
    =#

    if !isnothing(rotor_names); params_post_plot_rotor_sweep[5] .= rotor_names; end
    # prepare plot directory
    if !isdir(plot_directory); mkpath(plot_directory); end
    println("==== MSG ====\n\tplot_directory = $plot_directory")

    # build parameters struct
    parameters = VLM_BEM(params_solve_vlm_bem..., params_post_plot_lift_moment_distribution[3], params_post_plot_rotor_sweep[5], plot_directory, plot_base_name, plot_extension, plotstepi)

    # build freestream_function
    function freestream_function(aircraft, parameters, environment, alphas, stepi)

        # calculate freestream
        vinf = vinfs[stepi] # + ti # arbitrary for lift distribution?
        alpha = alphas[stepi]
        beta = 0.0
        Omega = StaticArrays.@SVector zeros(3)
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
    postactions = [post_plot_cf_cm_alpha_sweep, post_plot_lift_moment_distribution, post_plot_rotor_sweep]

    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, alphas) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, alphas, step_symbol
end

function vlm_bem_template(vinfs, plotstepi, alphas,
    wingsystem::VortexLatticeSystem,
    omegas, nblades_list, rhub_list, rtip_list, radii_list, chords_list, twists_list,
    contour_paths_list, index,
    rotor_positions, rotor_orientations, spin_directions, Res_lists, Ms_lists;
        wake_developement_factor = 1.0, # fully developed by default
        swirl_recovery_factor = 0.5, # as described in Veldhuis' paper
        surfacenames = ["default wing"],
        rotor_names = ["rotor 1"],
        mach_correction = nothing,
        mass = 0.0,
        inertia_x = 0.0,
        inertia_y = 0.0,
        inertia_z = 0.0,
        plot_directory = joinpath(topdirectory, "data","plots",TODAY),
        plot_base_name = "default",
        plot_extension = ".pdf",
        step_symbol = L"\alpha ",
        kwargs...
)
    # prepare subsystems
    rotors = CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
        radii_list, chords_list, twists_list,
        Res_lists, Ms_lists, contour_paths_list,
        index, rotor_positions, rotor_orientations, spin_directions;
            kwargs...
    )
    # rotors = CCBladeSystem(nblades, rhub, rtip, radii, chords, twists, contour_paths_list, index, rotor_positions, rotor_orientation, spin_directions, Res_list, Ms_list; kwargs...)

    return vlm_bem_template(vinfs, plotstepi, alphas, omegas,
        wingsystem, rotors;
            wake_developement_factor, # fully developed by default
            swirl_recovery_factor, # as described in Veldhuis' paper
            surfacenames,
            rotor_names,
            mass,
            inertia_x,
            inertia_y,
            inertia_z,
            plot_directory,
            plot_base_name,
            plot_extension,
            step_symbol,
            kwargs...
    )
end

function vlm_bem_template(vinfs, plotstepi, alphas,
    wing_lexs, wing_leys, wing_lezs, wing_chords, wing_thetas, wing_phis, wing_nspanwisepanels, wing_nchordwisepanels,
    omegas, nblades_list, rhub_list, rtip_list, radii, chords_list, twists_list,
    contour_paths_list, index,
    rotor_positions, rotor_orientations, spin_directions, Res_lists, Ms_lists;
        Vref = 1.0, symmetric = true, iref = 1, static_margin = 0.10, liftingline_x_over_c = 0.25,
        cambers = [fill((xc) -> 0, length(lex)) for lex in wing_lexs],
        spanwise_spacings = fill(VL.Uniform(), length(wing_lexs)),
        chordwise_spacings = fill(VL.Uniform(), length(wing_lexs)),
        wake_developement_factor = 1.0, # fully developed by default
        swirl_recovery_factor = 0.5, # as described in Veldhuis' paper
        surfacenames = ["default wing"],
        rotor_names = ["rotor 1"],
        mass = 0.0,
        inertia_x = 0.0,
        inertia_y = 0.0,
        inertia_z = 0.0,
        plot_directory = joinpath(topdirectory, "data","plots",TODAY),
        plot_base_name = "default",
        plot_extension = ".pdf",
        step_symbol = L"\alpha ",
        kwargs...
)
    # prepare subsystems
    wings = VortexLatticeSystem(wing_lexs, wing_leys, wing_lezs, wing_chords, wing_thetas, wing_phis, wing_nspanwisepanels, wing_nchordwisepanels; Vref, symmetric, iref, static_margin, liftingline_x_over_c, cambers, spanwise_spacings, chordwise_spacings)

    return vlm_bem_template(vinfs, plotstepi, alphas,
        wings,
        omegas, nblades_list, rhub_list, rtip_list, radii, chords_list, twists_list,
        contour_paths_list, index,
        rotor_positions, rotor_orientations, spin_directions, Res_lists, Ms_lists;
            wake_developement_factor, # fully developed by default
            swirl_recovery_factor, # as described in Veldhuis' paper
            surfacenames,
            rotor_names,
            mass,
            inertia_x,
            inertia_y,
            inertia_z,
            plot_directory,
            plot_base_name,
            plot_extension,
            step_symbol,
            kwargs...
    )
end
