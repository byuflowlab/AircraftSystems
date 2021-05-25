#=##############################################################################################
Filename: _common.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file contains structs common to multiple packages
=###############################################################################################

# system functions and types not specific to any imported module
abstract type RotorSystem end

abstract type WingSystem end

abstract type NonliftingBodySystem end

abstract type StructureSystem end

abstract type MotorSystem end

abstract type BatterySystem end

"""
Describes the freestream.

Inputs:

* `Vinf::Float64` : magnitude of the freestream
* `alpha::Float64` : angle of attack of the aircraft
* `beta::Float64` : sideslip angle of the aircraft
* `Omega::Vector{Float64}` : angular velocity about roll, pitch, and yaw axes

"""
struct Freestream{R}
    Vinf::Vector{R}
    Omega::Vector{R}
end

"""
Function describing a velocity field superimposed over the freestream. Dispatch is as follows:

function interstream(X::Vector{R})
    ...
    return velocity_at_X::Vector{R}
end

"""
abstract type Interstream end

"""
Contains environmental properties.

Inputs:

* `ρ::Float64` : freestream fluid density
* `ν::Float64` : freestream fluid kinematic viscocity
* `g::Float64` : acceleration due to gravity
* `T::Float64` : freestream fluid temperature
* `γ::Float64` : freestream fluid specific heat ratio
* `R::Float64` : freestream fluid gas constant
* `a::Float64` : speed of sound

"""
struct Environment{RT}
    ρ::RT
    ν::RT
    μ::RT
    g::RT
    T::RT
    γ::RT
    R::RT
    a::RT
end

"""
Convenience constructor for `Environment` struct.
"""
function Environment(;
        ρ = 1.225, # kg/m^3
        ν = 1.5e-5, # m^2/s
        μ = ν * ρ,
        g = 9.81, # m/s^2
        T = 288.15, # K = 15 deg C
        γ = 1.4,
        R = 287.05, # J/kg-K
        a = sqrt(γ * R * T)
    )
    return Environment(ρ, ν, μ, g, T, γ, R, a)
end
