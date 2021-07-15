#=##############################################################################################
Filename: system.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file defines environmental properties
=###############################################################################################

"""
    Environment{F}

Contains environmental properties.

# Fields:

* `ρ::F` : freestream fluid density
* `ν::F` : freestream fluid kinematic viscocity
* `μ::F` : freestream fluid dynamic viscosity
* `g::F` : acceleration due to gravity
* `T::F` : freestream fluid temperature
* `γ::F` : freestream fluid specific heat ratio
* `R::F` : freestream fluid gas constant
* `a::F` : speed of sound

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
    Environment(;
        ρ = 1.225, # kg/m^3
        ν = 1.5e-5, # m^2/s
        μ = ν * ρ, # kg/m-s
        g = 9.81, # m/s^2
        T = 288.15, # K = 15 deg C
        γ = 1.4,
        R = 287.05, # J/kg-K
        a = sqrt(γ * R * T))

Convenience constructor for `Environment` struct.

# Keyword Arguments

* `ρ`: air density
* `ν`: air kinematic viscosity
* `μ`: air dynamic viscosity
* `g`: gravity acceleration
* `T`: temperature (K)
* `γ`:
* `R`:
* `a`: speed of sound

"""
function Environment(;
        ρ=1.225, # kg/m^3
        ν=1.5e-5, # m^2/s
        μ=ν * ρ, # kg/m-s
        g=9.81, # m/s^2
        T=288.15, # K = 15 deg C
        γ=1.4,
        R=287.05, # J/kg-K
        a=sqrt(γ * R * T))

    return Environment(ρ, ν, μ, g, T, γ, R, a)
end
