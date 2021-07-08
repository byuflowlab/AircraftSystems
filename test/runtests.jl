import AircraftSystems
AS = AircraftSystems
import FileIO
FM = AS.FM
using LaTeXStrings
plt = AS.plt
# plt.pygui(false)
VL = AS.VL
using Test
LA = AS.LA

plotdirectory = "/Users/randerson/Box/research/projects/AircraftSystems/data/plots/$(AS.TODAY)"
notebookdirectory = ENV["NOTEBOOK_IMG_PATH"]

# @testset "rotors" begin

# include("epema_rotor_sweep.jl")

# include("rotor_wakefunction.jl")

# end # test rotors

# @testset "wings" begin

include("prowim_propsoff_clalpha.jl")

include("prowim_propsoff_liftdist.jl")

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

include("prowim_propson_liftdist.jl")

# end # test wings_and_rotors

