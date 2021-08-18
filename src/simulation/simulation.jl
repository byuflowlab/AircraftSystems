#=##############################################################################################
Filename: system.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this function is the top-level of the simulation
=###############################################################################################

"""
Runs a simulation.

Inputs:

* `aircraft::Aircraft` : `Aircraft` object defining the aircraft
* `parameters` : object containing additional parameters required by actions
* `actions::Vector{Action}` : contains functions inheriting from the `Action` type to be performed in chronological order each step
* `freestream_function::Function` : function returns a `::Freestream` object each step
* `environment_function::Function` : function returns a `::Environment` object each step
* `objective_function::Function` : function defines what is returned
* `step_range` : iterable object determining each step
* `step_symbol` : symbol defining the step, e.g. `alpha` or `time`

Outputs:

* `objective` : results of `objective_function(aircraft, parameters, freestream, environment, step_range)` after simulation

"""
function runsimulation!(aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, step_range, step_symbol;
        verbose = true
    )
    # begin simulation
    if verbose
        println("################################################################################################")
        println("                                       AIRCRAFT SYSTEMS")
        println("################################################################################################")
        println("\nRunning simulation...\n")
    end
    # step
    for (stepi, step) in enumerate(step_range)
        if verbose; println("\tstep: $stepi\t\t$step_symbol=$step"); end
        environment = environment_function(aircraft, parameters, step_range, stepi)
        freestream = freestream_function(aircraft, parameters, environment, step_range, stepi)
        for action! in actions
            flag = action!(aircraft, parameters, freestream, environment, step_range, stepi, step_symbol)
            if flag
                @warn "action $action failed at step=$stepi, $step_symbol=$step"
            end
        end
    end
    # perform post-processing
    if verbose; println("\nPerforming postprocessing...\n"); end
    for postaction! in postactions
        flag = postaction!(aircraft, parameters, step_range, step_symbol)
        if flag
            @warn "postaction $postaction failed"
        end
    end
    if verbose; println("\nSimulation complete.\n"); end
    # return objective
    return objective_function(aircraft, parameters, freestream, environment, step_range)
end
