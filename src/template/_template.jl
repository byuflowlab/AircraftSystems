"""
Functions that return all objects required for a particular kind of simulation. E.g.,

    ```julia
    function rotor_sweep_template() <: Template
        ...
        return system, parameters, actions, freestream_function, environment_function, objective_function, steprange
    end
    ```
"""
abstract type Template end
