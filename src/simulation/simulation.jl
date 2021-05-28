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
* `actions::Vector{Action}` : contains functions inheriting from the `Action` type to be performed in chronological order each time step
* `freestream_function::Function` : function returns a `::Freestream` object each timestep
* `environment_function::Function` : function returns a `::Environment` object each timestep
* `objective_function::Function` : function defines what is returned
* `timerange` : iterable object determining dimensional time of each step

Outputs:

* `objective` : results of `objective_function(aircraft, parameters, freestream, environment, timerange)` after simulation

"""
function runsimulation!(aircraft, parameters, actions, freestream_function, environment_function, objective_function, timerange)

    println("################################################################################################")
    println("                                       AIRCRAFT SYSTEMS")
    println("################################################################################################")
    println("\nRunning simulation...\n")
    # step in time
    for (ti, t) in enumerate(timerange)
        println("\ttimestep: $ti\tt=$t")
        environment = environment_function(aircraft, parameters, timerange, ti)
        freestream = freestream_function(aircraft, parameters, environment, timerange, ti)
        for action! in actions
            flag = action!(aircraft, parameters, freestream, environment, timerange, ti)
            if flag
                @warn "action $action failed at ti=$ti, t=$t"
            end
        end
    end

    return objective_function(aircraft, parameters, freestream, environment, timerange)
end
