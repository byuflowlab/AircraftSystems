"""
Defines a an `Action` object constructor.

Constructor inputs:

* `system::System`

Constructional returns:

* `myaction::Action` : an `Action` object

Dispatch is as follows:

```
function myconstructional(system) <: ActionConstructional
    ...
    return myaction
end
```

"""
abstract type ActionConstructor end

"""
Describes an action to be performed during simulation.

Contains:

* `activate::Function` : performs the action during simulation. Modifies `solution` and `parameters` and returns `false` upon successful completion. Dispatch is as follows:

    ```
    function action(solution::Dict, parameters::Dict, system::System, t::Float64)
        ...
        return false
    end
    ```

* `solutionkeys::Vector{String}` : each member is a key in the solution dictionary
* `solutioninits::Vector` : used to initialize the `solution` dictionary and improve memory allocation
* `parameterkeys::Vector{String}` : each member is a key in the `parameters` dictionary
* `parameterinits::Vector` : used to initialize the `parameters` dictionary and improve memory allocation

"""
struct Action
    activate!::Function
    solutionkeys::Vector{String}
    solutioninits::Vector
    parameterkeys::Vector{String}
    parameterinits::Vector
end

function Action(activate, solutionkeys, solutioninits, parameterkeys, parameterinits)
    @assert length(solutionkeys) == length(solutioninits) "length of `solutionkeys` and `solutioninits` inconsistent"
    @assert length(parameterkeys) == length(parameterinits) "length of `parameterkeys` and `parameterinits` inconsistent"
    Action(activate, solutionkeys, solutioninits)
end
