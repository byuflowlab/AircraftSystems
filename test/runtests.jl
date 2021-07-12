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
using Infiltrator

plotdirectory = joinpath(AS.topdirectory, "data/plots/$(AS.TODAY)")
contourdirectory = joinpath(AS.topdirectory, "test/data/airfoil/contours")
polardirectory = joinpath(AS.topdirectory, "test/data/airfoil/polars")

#! change this if you want to save figures from these tests. If so, edit the notebookdirectory below and the plot directory above.
const savefigs = false

if savefigs
    notebookdirectory = ENV["NOTEBOOK_IMG_PATH"]
end

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


# @testset "Epema Blown Wing Validation" begin

    # include("epema_blown_wing_cl.jl")
    # include("epema_blown_wing_cl_epema_polar.jl")

# end

# @testset "Veldhuis Blown Wing Validation" begin

#     include("veldhuis_blown_wing_cl.jl")

# end

# @testset "wings_and_rotors" begin

include("prowim_propson_liftdist.jl")

# end # test wings_and_rotors

