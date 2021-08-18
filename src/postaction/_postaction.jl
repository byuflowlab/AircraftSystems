#=##############################################################################################
Filename: _plot.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: after finishing a simulation, a list of ploting functions is executed
=###############################################################################################

"""
Describes a function used for postprocessing in `runsimulation!` with:

Inputs:

* `aircraft::Aircraft` : aircraft system struct
* `parameters <: Parameters` : inherits from the `Parameters` type; object containing data required by `<: Action` functions
* `step_range::AbstractArray` : range of times defining simulation
* `step_symbol::String` : defines the step, e.g. `alpha` or `time`

Modifies:

* `aircraft::Aircraft`
* `parameters <: Parameters`

Outputs:

* `flag::Bool` : true if an action experiences errors

Dispatch is as follows:

```
function mypostaction!(aircraft, parameters, step_range, step_symbol)
    ...
    return flag
end
```
"""
abstract type PostAction end
