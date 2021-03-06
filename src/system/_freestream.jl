#=##############################################################################################
Filename: system.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file defines freestream and interstream properties
=###############################################################################################

"""
    Freestream{F, VF <: AbstractArray}

Describes the freestream.

# Arguments:

* `Vinf::Float64` : magnitude of the freestream
* `alpha::Float64` : angle of attack of the aircraft
* `beta::Float64` : sideslip angle of the aircraft
* `Omega::Vector{Float64}` : angular velocity about roll, pitch, and yaw axes

"""
struct Freestream{TF1,TF2,TF3,TF4}
    vinf::TF1
    alpha::TF2
    beta::TF3
    Omega::StaticArrays.SArray{Tuple{3},TF4,1,3}
end

"""
    freestream2vector(freestream)

# Argument:

* `freestream::Freestream`: freestream properties

# Returns:

* `Vinf::Vector{Float64}`: freestream velocity in the body frame (aft-starboard-up)

"""
function freestream2vector(freestream)

    vinf = LA.norm(freestream.vinf)
    alpha = freestream.alpha
    beta = freestream.beta
    Vinf = vinf .*
        [   cos(alpha) * cos(beta),
            -sin(beta),
            sin(alpha) * cos(beta)
        ]

    return Vinf
end
