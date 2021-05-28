"""
Describes a function used in `runsimulation!` with:

Inputs:

* `system::System` : aircraft system struct
* `parameters <: Parameters` : inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `freestream::Freestream` : freestream object
* `environment::Environment` : environment object
* `timerange::AbstractArray` : range of times defining simulation
* `ti::Int` : index of the current timestep

Modifies:

* `system::System`
* `parameters <: Parameters`

Outputs:

* `flag::Bool` : true if an action experiences errors

Dispatch is as follows:

```
function myaction!(system, parameters, freestream, environment, timerange, ti) <: Action
    ...
    return flag
end
```
"""
abstract type Action end
