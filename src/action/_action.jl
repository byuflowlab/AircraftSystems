"""
Describes a function used in `runsimulation!` with:

Inputs:

* `aircraft::Aircraft` : aircraft system struct
* `parameters <: Parameters` : inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `freestream::Freestream` : freestream object
* `environment::Environment` : environment object
* `steprange::AbstractArray` : range of steps defining simulation
* `stepi::Int` : index of the current step
* `stepsymbol::String` : defines the step, e.g. `alpha` or `time`

Modifies:

* `aircraft::Aircraft`
* `parameters <: Parameters`

Outputs:

* `flag::Bool` : true if an action experiences errors

Dispatch is as follows:

```
function myaction!(aircraft, parameters, freestream, environment, steprange, stepi, stepsymbol) <: Action
    ...
    return flag
end
```
"""
abstract type Action end
