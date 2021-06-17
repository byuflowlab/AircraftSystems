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
* `steprange` : iterable object determining each step
* `stepsymbol` : symbol defining the step, e.g. `alpha` or `time`

Outputs:

* `objective` : results of `objective_function(aircraft, parameters, freestream, environment, steprange)` after simulation

"""
function runsimulation!(aircraft, parameters, actions, freestream_function, environment_function, postactions, objective_function, steprange, stepsymbol;
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
    for (stepi, step) in enumerate(steprange)
        if verbose; println("\tstep: $stepi\t\t$stepsymbol=$step"); end
        environment = environment_function(aircraft, parameters, steprange, stepi)
        freestream = freestream_function(aircraft, parameters, environment, steprange, stepi)
        for action! in actions
            flag = action!(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol)
            if flag
                @warn "action $action failed at step=$stepi, $stepsymbol=$step"
            end
        end
    end
    # perform post-processing
    if verbose; println("\nPerforming postprocessing...\n"); end
    for postaction! in postactions
        flag = postaction!(aircraft, parameters, steprange, stepsymbol)
        if flag
            @warn "postaction $postaction failed"
        end
    end
    if verbose; println("\nSimulation complete.\n"); end
    # return objective
    return objective_function(aircraft, parameters, freestream, environment, steprange)
end
