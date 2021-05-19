module AircraftSystems

# import packages
import CCBlade
CC = CCBlade
import Dates
import DelimitedFiles
DF = DelimitedFiles
import FLOWMath
FM = FLOWMath
using LaTeXStrings
import PyPlot
plt = PyPlot
import VortexLattice
VL = VortexLattice
import Xfoil
XF = Xfoil

# set path
const topdirectory = normpath(joinpath(@__DIR__, ".."))

# set global variables
global NU = 1.5e-5 # kinematic viscocity
global TODAY = replace(string(Dates.today()),"-" => "")

# load modules
include("system.jl")
include("analysis.jl")
include("rotor.jl")
include("plots.jl")


# export functions

end # module
