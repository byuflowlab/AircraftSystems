import AircraftSystems
AS = AircraftSystems
import FileIO
FM = AS.FM
using LaTeXStrings
plt = AS.plt
# plt.pygui(false)
VL = AS.VL
# using Test

plotdirectory = "/Users/randerson/Box/research/projects/AircraftSystems/data/plots/$(AS.TODAY)"
notebookdirectory = ENV["NOTEBOOK_IMG_PATH"]
if !isdir(notebookdirectory); mkpath(notebookdirectory); end

# @testset "rotors" begin

include("epema_rotor_sweep.jl")

# include("rotor_wakefunction.jl")

# end # test rotors

# @testset "wings" begin

# include("prowim_propsoff_clalpha.jl")

# include("prowim_propsoff_liftdist.jl")

# end # test wings




# @testset "wings_and_rotors" begin

# include("prowim_propson_liftdist.jl")

# end # test wings_and_rotors



# @testset "Epema Blown Wing Validation" begin

# include("epema_liftdist.jl")

# end # Epema Blown Wing Validation
