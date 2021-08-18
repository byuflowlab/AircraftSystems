#=##############################################################################################
Filename: inertia.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file provides inertial data and convenient manipulation functions
=###############################################################################################

struct Inertia{TF}
    mass::TF
    inertia_x::TF
    inertia_y::TF
    inertia_z::TF
end
