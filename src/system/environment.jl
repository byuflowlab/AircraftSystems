#=##############################################################################################
Filename: system.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file defines environmental properties
=###############################################################################################

"""
Contains environmental properties.

Inputs:

* `ρ::Float64` : freestream fluid density
* `ν::Float64` : freestream fluid kinematic viscocity
* `μ::Float64` : freestream fluid dynamic viscosity
* `g::Float64` : acceleration due to gravity
* `T::Float64` : freestream fluid temperature
* `γ::Float64` : freestream fluid specific heat ratio
* `R::Float64` : freestream fluid gas constant
* `a::Float64` : speed of sound

"""
struct Environment{F}
    ρ::F
    ν::F
    μ::F
    g::F
    T::F
    γ::F
    R::F
    a::F
end

"""
Convenience constructor for `Environment` struct.
"""
function Environment(;
        ρ = 1.225, # kg/m^3
        ν = 1.5e-5, # m^2/s
        μ = ν * ρ, # kg/m-s
        g = 9.81, # m/s^2
        T = 288.15, # K = 15 deg C
        γ = 1.4,
        R = 287.05, # J/kg-K
        a = sqrt(γ * R * T)
    )
    return Environment(ρ, ν, μ, g, T, γ, R, a)
end
