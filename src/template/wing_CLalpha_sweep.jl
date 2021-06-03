#=##############################################################################################
Filename: wing_CLalpha_sweep.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this is a template file. Convenience methods are provided to prepare a single rotor and
        sweep over advance ratios.
=###############################################################################################

# initialize parameters
struct WingCLalphaParameters{V1,V2,V3,V4,V5,V6} <: Parameters
    omegas::V1
    alphas::V2
    CTs::V3
    CQs::V4
    ηs::V5
    alphas_history::V6
end

# function (rs::RotorSweepParameters{V1, V2, V3, V4, V5})(alphas, omega...)

# end

# rs  = RotorSweepParameter(args...)

# rs(alphas, omega, nblades...)

function wing_CLalpha_sweep_template(alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip; kwargs...)
    # prepare subsystems
    wings = simplewing(; wing_b = 2.0, wing_TR = 0.8, wing_AR = 8.0, wing_θroot = 0.0, wing_θtip = 0.0, kwargs...)
    rotors = nothing
    nonliftingbodies = nothing
    structures = nothing
    motors = nothing
    batteries = nothing

    # build system struct
    aircraft = Aircraft(wings, rotors, nonliftingbodies, structures, motors, batteries)

    # compile actions
    actions = [solve_wing]

    # initialize parameters
    timerange = 1:length(alphas) # use time to define each part of the sweep
    omegas_zero, alphas_zero, CTs, CQs, ηs = solve_wing(aircraft, timerange)

    # check sizes and instantiate struct
    @assert size(omegas_zero) == size(omegas) "`omegas` has improper dimensions. \n\tExpected: $(size(omegas_zero))\n\tGot: $(size(omegas))"
    @assert size(alphas_zero) == size(alphas) "`alphas` has improper dimensions. \n\tExpected: $(size(alphas_zero))\n\tGot: $(size(alphas))"
    parameters = RotorSweepParameters(omegas, alphas, CTs, CQs, ηs, alphas_zero)

    # build freestream_function
    function freestream_function(aircraft, parameters, environment, timerange, ti)
        J = parameters.alphas[ti][1] # J is set on the first rotor only
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
