#=##############################################################################################
Filename: system.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file defines the `System` struct.
=###############################################################################################


"""
Aircraft system. Types of all members must inherit from their respective abstract types.

`System` objects are meant to be flexible as to what types populate their members. This
allows simulations to be run with user-designed models. Functionality is defined in
instances of the `Action` and `Observer` types.

Attributes:

* `wings` : comprehensively defines the aircraft's wings
* `rotors` : comprehensively defines the aircraft's rotors
* `nonliftingbodies` : comprehensively defines non-lifting body(s) composing the aircraft
* `structures` : comprehensively defines the aircraft's structure(s)
* `motors` : comprehensively defines the aircraft's motor(s)
* `batteries` : comprehensively defines the aircraft's batterie(s)

"""
struct System
    wings::WingSystem
    rotors::RotorSystem
    nonliftingbodies::NonliftingBodySystem
    structures::StructureSystem
    motors::MotorSystem
    batteries::BatterySystem
end
