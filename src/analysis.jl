# analysis functions

"Calculates lift, drag, and moment coefficients of the system.

Inputs:

* `system::System : `BlownWing.System` object
* `freestream::VL.Freestream` : freestream information
* `control::Vector{R} : vector of rotor RPM control inputs

Outputs:

* no explicit outputs, but cl, cd, cm, CL, CD, and CM values are computed and stored inside `system.VL_system`.

"
function solve!(system::System, freestream::VL.Freestream, control::Vector{R}) where R

end
