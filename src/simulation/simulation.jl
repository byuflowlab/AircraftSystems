"""
Runs a simulation.

Inputs:

* `system::System` : `System` object defining the aircraft
* `actions::Vector{Action}` : contains `Action` structs to be performed in chronological order each time step
* `timerange` : iterable object determining dimensional time of each step

"""
function runsimulation(system, actions::Vector{Action}, timerange)
    # avoid state issues by copying `system`
    thissystem = deepcopy(system)

    # prepare parameters dictionary
    parameters = Dict() # Dictionaries are very slow! Look into typing
    for action in actions
        for (ikey, key) in enumerate(action.parameterkeys)
            if key !in(parameters.keys)
                parameters[key] = action.parameterinits[ikey]
            else
                @warn "parameter key \"$key\" is already initialized; using the first instance"
            end
        end
    end

    # prepare solution dictionary
    solution = Dict()
    for action in actions
        for (ikey, key) in enumerate(action.solutionkeys)
            if key !in(solution.keys)
                solution[key] = action.solutioninits[ikey]
            else
                @warn "solution key \"$key\" is already initialized; using the first instance"
            end
        end
    end

    # step in time
    for (ti, t) in enumerate(timerange)
        for action in actions
            flag = action.activate!(solution, parameters, thissystem, t)
            if flag
                @warn "action $action failed at t=$t"
            end
        end
    end

    return solution
end
