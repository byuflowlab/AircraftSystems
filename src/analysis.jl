# analysis functions

"Calculates overall lift, drag, moment, and rotor coefficients of the system.

Inputs:

* `system::System : `BlownWing.System` object
* `freestream::VL.Freestream` : freestream information
* `control::Vector{R} : vector of rotor RPM control inputs

Outputs:

* CF::Vector{Array{R,1}} : each vector element is a 1-D array corresponding to each lifting surface in `system.system` and containing [C_D, C_L, C_S]
* CM::Vector{Array{R,1}} : each vector element is a 1-D array corresponding to each lifting surface in `system.system` and containing [C_Mx, C_My, C_Mz]
* CRotor::Vector{Tuple{R,R,R}} : each vector element is a 3-Tuple corresponding to each rotor and containing (C_T, C_Q, eta)

"
function solve!(system::System, freestream::VL.Freestream, control::Vector{R}) where R

end

"
Returns local lift, drag, and moment coefficients of the system.

Outputs:

* cf::Vector{Array{R,2},1} : each vector element is a 2-D array corresponding to each lifting surface in `system.system` and containing force coefficients (cd, cl, cs) for each spanwise station
* cm::Vector{Array{R,2},1} : each vector element is a 2-D array corresponding to each lifting surface in `system.system` and containing moment coefficients for each spanwise station
"
function forcedistribution(system::System)

end
