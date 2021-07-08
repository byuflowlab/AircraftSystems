import AircraftSystems
AS = AircraftSystems
import FileIO
FM = AS.FM
using LaTeXStrings
plt = AS.plt
# plt.pygui(false)
VL = AS.VL
using Test
using PyPlot
using Infiltrator
import LinearAlgebra
LA = AS.LA

# @testset "rotors" begin

# include("epema_rotor_sweep.jl")

# end # test rotors

# @testset "wings" begin

# include("prowim_propsoff_clalpha.jl")

# include("prowim_propsoff_liftdist.jl")

# end # test wings


# @testset "Epema Wing Validation" begin

# include("epema_liftdist.jl")

# end # Epema Blown Wing Validation


@testset "Epema Blown Wing Validation" begin

    include("epema_blown_wing_cl.jl")

end

# @testset "Veldhuis Blown Wing Validation" begin

#     include("veldhuis_blown_wing_cl.jl")

# end

# @testset "wings_and_rotors" begin

# include("proowim_propson_liftdist.jl")

# test rotor wake
# include("rotor_wakefunction.jl")

# end # test wings_and_rotors

