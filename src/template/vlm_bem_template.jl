#=##############################################################################################
Filename: blown_wing_template.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to calculate the lift
        distribution of a blown wing over the specified alphas.
=###############################################################################################

# initialize parameters
struct VLM_BEM{V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11} <: Parameters
    omegas::V1
    Js::V2
    Ts::V3
    Qs::V3
    us::V4
    vs::V4
    wakefunctions::V5
    CLs::V6
    CDs::V6
    CYs::V6
    cls::V7
    cds::V7
    cys::V7
    cmxs::V7
    cmys::V7
    cmzs::V7
    cfs::V8
    cms::V8
    surfacenames::V9
    plotdirectory::V10
    plotbasename::V10
    plotextension::V10
    plotstepi::V11
end

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(alphas, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(alphas, omega, nblades...)

"""
vlm_bem_template <: Template

Inputs:

* `alphas::Vector{Float64}`
* `wing_b::Float64`
* `wing_TR::Float64`
* `wing_AR::Float64`
* `wing_θroot::Float64`
* `wing_θtip::Float64`

Keyword Arguments:

* `plotdirectory = joinpath(topdirectory, "data","plots")`: directory to which plots are saved
* `plotbasename = "default"`: first part of saved plot filenames
* `plotextension = ".pdf"`: saved plots file extension

"""
function vlm_bem_template(plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, rotor_X, rotor_orientation, spindirections, Res_list = [fill([5e4, 1e5, 1e6], length(radii))];
    surfacenames = ["default wing"],
    rotornames = ["rotor 1"],
    plotdirectory = joinpath(topdirectory, "data","plots",TODAY),
    plotbasename = "default",
    plotextension = ".pdf",
    stepsymbol = L"\alpha ",
    kwargs...
);
    # prepare subsystems
    wings = simplewingsystem(wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip; kwargs...)
    rotors = CCBladeSystem(nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, rotor_X, rotor_orientation, spindirections, Res_list = Res_list; kwargs...)
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
    #= Contains:
        * `omegas::Vector{Float64}` : a vector of length `length(steprange)` containing a vector of rotational velocities for each rotor
        * `Js::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing a vector of advance ratios for each rotor
        * `Ts::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing a vector of dimensional thrust values for each rotor
        * `Qs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing a vector of dimensional torque values for each rotor
        * `us::Vector{Vector{Float64}}` : each [i][j,k]th element is the axial induced velocity at the jth radial station of the ith rotor at the kth step
        * `vs::Vector{Vector{Float64}}` : each [i][j,k]th element is the swirl induced velocity at the jth radial station of the ith rotor at the kth step
        * `wakefunction::Function` : function accepts position X and returns the rotor induced velocity
        * `CLs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing lift coefficients at each step
        * `CDs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing drag coefficients at each step
        * `CYs::Vector{Vector{Float64}}` : a vector of length `length(steprange)` containing side force coefficients at each step
        * `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
        * `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
        * `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
        * `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
        * `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
        * `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface
    =#
    params_plot_cl_alpha_sweep = plot_cl_alpha_sweep(aircraft, steprange)
    #= Contains:
        * `CLs::Vector{Float64}` : CL corresponding to the ith step
        * `CDs::Vector{Float64}` : CD corresponding to the ith step
        * `CYs::Vector{Float64}` : CY corresponding to the ith step
        * `plotdirectory::String` : directory where plots are saved
        * `plotbasename::String` : first portion of the saved figure file name
        * `plotextension::String` : extension of saved figure files
    =#
    params_plot_lift_moment_distributions = plot_lift_moment_distributions(aircraft, steprange)
    #= Contains:
        * `cls::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local lift coefficients at each lifting line section, corresponding to each lifting surface
        * `cds::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local drag coefficients at each lifting line section, corresponding to each lifting surface
        * `cys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local side force coefficients at each lifting line section, corresponding to each lifting surface
        * `cmxs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local x-axis (roll) moment coefficients at each lifting line section, corresponding to each lifting surface
        * `cmys::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local y-axis (pitch) moment coefficients at each lifting line section, corresponding to each lifting surface
        * `cmzs::Vector{Array{Float64,2}}` : each element is an array of size (nspanwisepanels, nsteps) containing local z-axis (yaw) moment force coefficients at each lifting line section, corresponding to each lifting surface
        * `cfs::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of force coefficients cd, cy, cl
        * `cms::Vector{Array{Float64,2}}` : vector with length equal to the number of lifting surfaces, each member containing an array of size (3,nspanwisepanels) of moment coefficients cmx, cmy,
        * `surfacenames::Vector{String}` : names of each lifting surface to be shown in the legend
        * `plotdirectory::String` : path to the folder where plots will be saved
        * `plotbasename::String` : first part of saved figure file names
        * `plotextension::String` : extension of saved figure file names
        * `plotstepi::Vector{Int}` : which steps at which to plot
    =#
    # prepare plot directory
    if !isdir(plotdirectory); mkpath(plotdirectory); end
    println("==== MSG ====\n\tplotdirectory = $plotdirectory")
    # build parameters struct
    parameters = VLM_BEM(params_solve_vlm_bem..., params_plot_lift_moment_distributions[7:9]..., plotdirectory, params_plot_lift_moment_distributions[11:12]..., plotstepi)
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
    postactions = [plot_cl_alpha_sweep, plot_lift_moment_distributions]
    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, alphas) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, alphas, stepsymbol
end
