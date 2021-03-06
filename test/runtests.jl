println("Begin importing packages for `runtests.jl`...")
import AircraftSystems
AS = AircraftSystems
CC = AS.CC
FM = AS.FLOWMath
LS = AS.LaTeXStrings
LA = AS.LinearAlgebra
plt = AS.PyPlot
# plt.pygui(false)
T = AS.Test
VL = AS.VortexLattice
println("Finished.")

plot_directory = joinpath(AS.topdirectory, "data", "plots", AS.TODAY)
if !isdir(plot_directory); mkpath(plot_directory); end

const save_figs = false #! Set true if you need to save the test figures.

# prepare Epema rotor
include("EpemaData.jl")
contour_directory = joinpath(AS.topdirectory, "test", "data", "airfoil", "contours")
polar_directory = joinpath(AS.topdirectory, "test", "data", "airfoil", "polars")
rs_desired = [0.207, 0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0]
# EpemaData.scans_to_interpolated_contours(contour_directory, plot_directory, rs_desired)

# Fetch PROWIM data.
include("PROWIMData.jl")

# T.@testset "rotors" begin

include("epema_rotor_sweep.jl")

include("rotor_wakefunction.jl") #currently doesn't work for me, I think the parameters need adjusting first but I'll let you handle it

# end # test rotors

# T.@testset "wings" begin

include("prowim_propsoff_clalpha.jl")

include("prowim_propsoff_liftdist.jl")

# include("prowim_test_cl_CL.jl")

# end

# @testset "Epema Wing Validation" begin

# include("epema_liftdist.jl")

# end # Epema Blown Wing Validation

# T.@testset "Epema Blown Wing Validation" begin #* still a work in progress. No errors that keep it from running, just not great results yet.

    # include("epema_blown_wing_cl.jl")
    include("epema_blown_wing_cl_epema_polar.jl") # uses the Epema polars from above

# end

# T.@testset "Veldhuis Blown Wing Validation" begin

include("prowim_propson_liftdist.jl")

# end

# benchmarks

include("benchmark_ccblade.jl")
