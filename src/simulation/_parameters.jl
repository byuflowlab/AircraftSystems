#=##############################################################################################
Filename: system.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file defines the `Parameters` type.
=###############################################################################################

"""
Objects inheriting from the `Parameters` type contain dot indexed members and are passed into action functions. A single `<: Parameters` struct is defined for the entire simulation and must contain members required by all Action functions. Action functions are overloaded to accept an `Aircraft` struct and a `step_range::AbstractArray` object and return default values.

For example:

struct MyParameters{ta, tb} <: Parameters
    a::ta
    b::tb
end

"""
abstract type Parameters end
