module AircraftSystems

# import packages
import CCBlade
CC = CCBlade
import Dates
import DelimitedFiles
DF = DelimitedFiles
import FLOWMath
FM = FLOWMath
# import GeometricTools
# GT = GeometricTools
using LaTeXStrings
import LinearAlgebra
LA = LinearAlgebra
import PyPlot
plt = PyPlot
import VortexLattice
VL = VortexLattice
import Xfoil
XF = Xfoil

# set path
const topdirectory = normpath(joinpath(@__DIR__, ".."))

# set date
const TODAY = replace(string(Dates.today()),"-" => "")

# load files
subdirectories = [
    "system/",
    "action/fundamental/",
    "action/compound/",
    "postaction/",
    "simulation/",
    "template/",
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

# export packages
export CCBlade, FLOWMath, VortexLattice, Xfoil

# export functions
export runsimulation!

end # module
