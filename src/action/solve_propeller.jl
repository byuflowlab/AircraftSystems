#=##############################################################################################
Filename: solve_rotor.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: define an `Action` object to solve a CCBlade rotor
=###############################################################################################

function solve_propeller(system::System, timerange) <: ActionConstructor

    # prepare parameterkeys and parameterinits for this particular system and simulation
    parameterkeys = [
        "freestream",
        "environment",
        "omegas"
    ]

    freestream_alpha0 = 0.0
    freestream_beta0 = 0.0
    freestream_omega0 = [0.0,0.0,0.0]
    freestream_magnitude0 = 1.0

    omegas0 = ones(length(system.rotorsystem.index))

    parameterinits = [
        Freestream(freestream_alpha0, freestream_beta0, freestream_omega0, freestream_magnitude0),
        Environment(),
        omegas0
    ]

    # prepare solutionkeys and solutioninits for this particular system and simulation
    solutionkeys = [
        "thrust",
        "torque",
        "efficiency",
        "u",
        "v"
    ]

    nrotors = length(system.RotorSystem.index)
    nsections = [length(sectionlist) for sectionlist in system.RotorSystem.sectionlists]
    u_init0 = [zeros(nsections[i]) for i in system.RotorSystem.index]
    u_init = fill(u_init0, length(timerange))
    v_init = deepcopy(u_init)

    solutioninits = [
        fill([0.0,0.0,0.0], length(timerange)),
        zeros(length(timerange)),
        zeros(length(timerange)),
        u_init,
        v_init
    ]

    # define `activate!` function
    function activate!(solution::Dict, parameters::Dict, system::System, ti)
        CC_freestream = parameters["freestream"]
    end

    return Action(activate!, solutionkeys, solutioninits, parameterkeys, parameterinits)
end

function solverotorsystem(rotorsystem::CCBladeSystem, omegas::Vector, freestream::Freestream, environment::Environment)#, interstream::Interstream)

    operatingpoints_list = CC.OperatingPoint(rotorsystem, omegas, freestream, environment)
    for (i, rotorindex) in enumerate(rotorsystem.index)
        rotor = rotorsystem.rotors[rotorindex]
        sections = rotorsystem.sectionlists[rotorindex]
        operatingpoints = operatingpoints_list[i]
        outputs = CC.solve.(Ref(rotor), sections, operatingpoints)
        T, Q = CC.thrusttorque(rotor, sections, outputs)

        η, CT, CQ = CC.nondim(T, Q, vhubs[i], omegas[i], environment.ρ, rotor, "propeller")

    end
end
