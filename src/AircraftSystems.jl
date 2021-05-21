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

# load files
subdirectories = [
    "system/",
    "interactions/",
    "observers/",
    "templates/",
    "simulation/"
]
for directory in subdirectories
    for (root, dirs, files) in walkdir(joinpath(topdirectory,"src",directory))
        for file in files
            if splitext(file)[end] == ".jl" #&& file != "TrajOpt2020.jl" && !(file in excluded)
            # println("file = ",joinpath(root, file))
                include(joinpath(root,file))
            end
        end
    end
end

# export functions


end # module
