#=##############################################################################################
Filename: system.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file defines the `Aircraft` system struct.
=###############################################################################################


"""
    Aircraft{w,r,n,s,m,b}

`Aircraft` system object.

Members are subsystems of the aircraft. Subsystems are not typed to provide flexibility as to what models are used. This
allows simulations of a wide range of interchangeable models. Functionality is defined in
instances of the `Action` types.

# Fields:

* `wingsystem` : comprehensively defines the aircraft's wings
* `rotorsystem` : comprehensively defines the aircraft's rotors
* `nonliftingbodysystem` : comprehensively defines non-lifting body(s) composing the aircraft
* `structuresystem` : comprehensively defines the aircraft's structure(s)
* `motorsystem` : comprehensively defines the aircraft's motor(s)
* `batterysystem` : comprehensively defines the aircraft's batterie(s)

"""
struct Aircraft{TF,TF1,TF2,TF3,TAF}
        # T1 <: Union{Nothing, CC.MachCorrection}, T2 <: Union{Nothing, CC.ReCorrection},
        # T3 <: Union{Nothing, CC.RotationCorrection}, T4 <: Union{Nothing, CC.TipCorrection}}
    wingsystem::Union{VortexLatticeSystem{TF}, Nothing}
    rotorsystem::CCBladeSystem{TF,TF1,TF2,TF3,TAF}#,T1,T2,T3,T4}, Nothing}
    inertiasystem::Union{Inertia{TF}, Nothing}
    nonliftingbodysystem::Nothing
    structuresystem::Nothing
    motorsystem::Nothing
    batterysystem::Nothing
end

CCBladeSystem{Float64,Float64,Float64,Float64,CCBlade.AlphaReMachAF{Float64,String}}
