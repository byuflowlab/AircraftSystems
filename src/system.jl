"Blown wing system."
struct System{R}
    # CCBlade objects
    rotors::Vector{CC.Rotor}
    # VortexLattice objects
    VL_system::VL.System
    VL_reference::VL.Reference
    # coupling
    rotorindex::Vector{R}                   # which element of `System.rotors`
    rotorposition::Vector{Tuple{R,R,R}}     # position of each rotor
    rotororientation::Vector{Array{R,2}}    # forward axial orientation of each rotor
    wakedevelopmentfactor::Vector{R}        # 0-1: set how developed each wake is
end

"Constructor for `System` object."
function System()

end
