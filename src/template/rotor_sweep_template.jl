#=##############################################################################################
Filename: rotor_sweep.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to prepare a single rotor and
        sweep over advance ratios.
=###############################################################################################

# initialize parameters
struct RotorSweepParameters{V1,V2,V3,V4,V5,V6,V7} <: Parameters
    omegas::V1
    Js::V2
    CTs::V3
    CQs::V4
    ηs::V5
    rotornames::V6
    plotdirectory::V7
    plotbasename::V7
    plotextension::V7
end

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(Js, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(Js, omega, nblades...)

# build freestream_function
function get_J(parameters::RotorSweepParameters{V1,V2,V3,V4,V5,V6}, ri, stepi) where V1 where V2 <:AbstractArray{<:Any,1} where V3 where V4 where V5 where V6
    parameters.Js[stepi]
end

function get_J(parameters::RotorSweepParameters{V1,V2,V3,V4,V5,V6}, ri, stepi) where V1 where V2 <:AbstractArray{<:Any,2} where V3 where V4 where V5 where V6
    parameters.Js[ri,stepi]
end

"""
rotor_sweep_template <: Template

Inputs:

* `Js::Array{Float64,2}` : each [i,j]th element contains the desired advance ratio of the ith rotor at the jth step
* `omegas::Array{Float64,2}` : each [i,j]th element contains the commanded rotational velocity in rad/s of the ith rotor at the jth step
* `nblades::Vector{Int}` : number of blades of the ith rotor
* `rhub::Vector{Float64}` : hub radius of the ith rotor
* `rtip::Vector{Float64}` : tip radius of the ith rotor
* `radii::Vector{Vector{Float64}}` : ith element is a vector of radial stations defining the ith rotor
* `chords::Vector{Vector{Float64}}` : ith element is a vector of chords defining the ith rotor
* `twists::Vector{Vector{Float64}}` : ith element is a vector of twist angles in radians defining the ith rotor
* `airfoilcontours::Vector{Vector{String}}` : ith element is a vector of paths to contour file defining the ith rotor
* `airfoilnames::Vector{Vector{String}}` : ith element is a vector of strings defining the airfoils at each radial section of the ith rotor

"""
function rotor_sweep_template(Js, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, Res_list = [fill([5e4, 1e5, 1e6], length(radii))];
        rotornames = ["rotor 1"],
        plotdirectory = joinpath(topdirectory, "data","plots",TODAY),
        plotbasename = "default",
        plotextension = ".pdf",
        stepsymbol = L"J",
        kwargs...
    )
    # prepare subsystems
    wings = nothing
    rotors = CCBladeSystem([nblades], [rhub], [rtip], [radii], [chords], [twists], [airfoilcontours], [airfoilnames], [1], [[0.0,0.0,0.0]], [[-1.0,0.0,0.0]], [false], Res_list = Res_list; kwargs...)
    nonliftingbodies = nothing
    structures = nothing
    motors = nothing
    batteries = nothing

    # build system struct
    aircraft = Aircraft(wings, rotors, nonliftingbodies, structures, motors, batteries)

    # compile actions
    actions = [solve_rotor]

    # initialize parameters
    steprange = 1:length(Js) # use time to define each part of the sweep
    omegas_zero, Js_zero, CTs, CQs, ηs = solve_rotor(aircraft, steprange)

    # check data
    @assert length(omegas_zero) == length(omegas) "`omegas` has improper dimensions. \n\tExpected: $(size(omegas_zero))\n\tGot: $(size(omegas))"
    @assert length(Js_zero) == length(Js) "`Js` has improper dimensions. \n\tExpected: $(size(Js_zero))\n\tGot: $(size(Js))"
    @assert isdir(plotdirectory) "plotdirectory does not exist at $plotdirectory\n\ttry: `mkdir $plotdirectory`"
    # build parameters
    parameters = RotorSweepParameters(omegas, Js, CTs, CQs, ηs, rotornames, plotdirectory, plotbasename, plotextension)

    function freestream_function(aircraft, parameters, environment, steprange, stepi)
        J = get_J(parameters, 1, stepi)
        omega = parameters.omegas[1] # get omega for the first rotor
        n = omega / 2 / pi
        D = aircraft.rotorsystem.rotors[1].Rtip * 2
        # calculate freestream
        Vinf = J * n * D
        alpha = 0.0
        beta = 0.0
        Omega = zeros(3)
        freestream = Freestream(Vinf, alpha, beta, Omega)
        return freestream
    end

    # build environment_function
    function environment_function(aircraft, parameters, steprange, stepi)
        Environment()
    end

    # assemble postactions
    postactions = [plot_rotorsweep]

    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, steprange) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, steprange, stepsymbol
end
