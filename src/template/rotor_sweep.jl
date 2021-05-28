#=##############################################################################################
Filename: rotor_sweep.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to prepare a single rotor and
        sweep over advance ratios.
=###############################################################################################

# initialize parameters
struct RotorSweepParameters{V1,V2,V3,V4,V5,V6} <: Parameters
    omegas::V1
    Js::V2
    CTs::V3
    CQs::V4
    ηs::V5
    Js_history::V6
end

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(Js, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(Js, omega, nblades...)

function rotor_sweep_template(Js, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, Res_list = [fill([5e4, 1e5, 1e6], length(radii))]; kwargs...)
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
    actions = [solve_propeller]

    # initialize parameters
    timerange = 1:length(Js) # use time to define each part of the sweep
    omegas_zero, Js_zero, CTs, CQs, ηs = solve_propeller(aircraft, timerange)

    # check sizes and instantiate struct
    @assert size(omegas_zero) == size(omegas) "`omegas` has improper dimensions. \n\tExpected: $(size(omegas_zero))\n\tGot: $(size(omegas))"
    @assert size(Js_zero) == size(Js) "`Js` has improper dimensions. \n\tExpected: $(size(Js_zero))\n\tGot: $(size(Js))"
    parameters = RotorSweepParameters(omegas, Js, CTs, CQs, ηs, Js_zero)

    # build freestream_function
    function freestream_function(aircraft, parameters, environment, timerange, ti)
        J = parameters.Js[ti][1] # J is set on the first rotor only
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
    function environment_function(aircraft, parameters, timerange, ti)
        Environment()
    end

    # build objective_function
    objective_function(aircraft, parameters, freestream, environment, timerange) = 0.0

    return aircraft, parameters, actions, freestream_function, environment_function, objective_function, timerange
end
