#=##############################################################################################
Filename: system.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file defines the `Aircraft` system struct.
=###############################################################################################


"""
`Aircraft` system object.

Members are subsystems of the aircraft. Subsystems are not typed to provide flexibility as to what models are used. This
allows simulations of a wide range of interchangeable models. Functionality is defined in
instances of the `Action` types.

Contains:

* `wingsystem` : comprehensively defines the aircraft's wings
* `rotorsystem` : comprehensively defines the aircraft's rotors
* `nonliftingbodysystem` : comprehensively defines non-lifting body(s) composing the aircraft
* `structuresystem` : comprehensively defines the aircraft's structure(s)
* `motorsystem` : comprehensively defines the aircraft's motor(s)
* `batterysystem` : comprehensively defines the aircraft's batterie(s)

"""
struct Aircraft{w,r,n,s,m,b}
    wingsystem::w
    rotorsystem::r
    nonliftingbodysystem::n
    structuresystem::s
    motorsystem::m
    batterysystem::b
end
