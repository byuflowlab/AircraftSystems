println("Begin importing packages for `runtests.jl`...")
using AircraftSystems
AS = AircraftSystems
FM = AS.FLOWMath
LA = AS.LA
# using LaTeXStrings
LS = LaTeXStrings
plt = AS.PyPlot
# plt.pygui(false)
# using Test
T = AS.Test
VL = AS.VortexLattice
println("Finished.")

plotdirectory = joinpath(AS.topdirectory, "data", "plots", AS.TODAY)
# notebookdirectory = ENV["NOTEBOOK_IMG_PATH"]
# if !isdir(notebookdirectory); mkpath(notebookdirectory); end

const savefigs = false #! switch if you need to save the test figures, otherwise leave alone

# prepare Epema rotor
include("EpemaData3.jl")
contourdirectory = joinpath(AS.topdirectory, "test", "data", "airfoil", "contours")
polardirectory = joinpath(AS.topdirectory, "test", "data", "airfoil", "polars")
rs_desired = [0.207, 0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0]
EpemaData3.scans_to_interpolated_contours(contourdirectory, plotdirectory, rs_desired)

T.@testset "rotors" begin

include("epema_rotor_sweep.jl")

# include("rotor_wakefunction.jl") #currently doesn't work for me, I think the parameters need adjusting first but I'll let you handle it

end # test rotors

T.@testset "wings" begin

include("prowim_propsoff_clalpha.jl")

include("prowim_propsoff_liftdist.jl")

end # test wings


# @testset "Epema Wing Validation" begin

# include("epema_liftdist.jl")

# end # Epema Blown Wing Validation

# T.@testset "wings_and_rotors" begin

T.@testset "Epema Blown Wing Validation" begin #* still a work in progress. No errors that keep it from running, just not great results yet.

    # include("epema_blown_wing_cl.jl")
    include("epema_blown_wing_cl_epema_polar.jl") # uses the Epema polars from above

end

# T.@testset "Veldhuis Blown Wing Validation" begin

#     include("veldhuis_blown_wing_cl.jl")

# end

T.@testset "wings_and_rotors" begin

include("prowim_propson_liftdist.jl")

end # test wings_and_rotors

