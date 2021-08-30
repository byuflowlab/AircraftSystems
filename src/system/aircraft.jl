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

* `wing_system` : comprehensively defines the aircraft's wings
* `rotor_system` : comprehensively defines the aircraft's rotors
* `nonlifting_body_system` : comprehensively defines non-lifting body(s) composing the aircraft
* `structure_system` : comprehensively defines the aircraft's structure(s)
* `motor_system` : comprehensively defines the aircraft's motor(s)
* `battery_system` : comprehensively defines the aircraft's batterie(s)

"""
struct Aircraft{TF,TF1,TF2,TF3,TAF}
        # T1 <: Union{Nothing, CC.Mach_correction}, T2 <: Union{Nothing, CC.ReCorrection},
        # T3 <: Union{Nothing, CC.RotationCorrection}, T4 <: Union{Nothing, CC.TipCorrection}}
    wing_system::Union{VortexLatticeSystem{TF}, Nothing}
    rotor_system::CCBladeSystem{TF,TF1,TF2,TF3,TAF}#,T1,T2,T3,T4}, Nothing}
    inertia_system::Union{Inertia{TF}, Nothing}
    nonlifting_body_system::Nothing
    structure_system::Nothing
    motor_system::Nothing
    battery_system::Nothing
end
