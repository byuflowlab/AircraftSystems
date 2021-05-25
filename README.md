# AircraftSystems

Authors: Ryan Anderson

Email: rymanderson@gmail.com

Date: 12 May 2021

# Purpose

Create a seamless interface between several intersecting codes applied to aerodynamic systems.

# Desired Features

* calculate lift, drag, and moments
* include rotor-on-wing interactions
* include wing-on-rotor interactions?
* plot lift distributions
* plot isolated rotor performance
* prepares airfoil data for rotors
* add feature: average including and excluding partially waked panels for optimization
* can we choose which packages it imports?

# Questions

* should the user create CCBlade and VortexLattice objects explicitly?

# Notes

* on-the-fly rotational corrections in CCBlade do not work

# Code Structure

* lifting surfaces -> defined in VortexLattice; convenience functions added to quickly build common designs
* VLM parameters -> optional parameters can be set here (struct** or dictionary)
* rotors -> defined in CCBlade; convenience functions added for common designs
* CCBlade parameters -> optional parameters can be set here (struct)
* structures -> defined in GXBeam (given stiffness and loads -> deflections) and PreComp (to get stiffness matrix -> GXBeam, see Design and Analysis of Composite Structures, p48) and CompositeStructures (representation of geometry/cross sections/etc.; buckling; test for ply failure for composite materials; inputs to PreComp)

    * in GXBeam- points are external forces; elements are internal -> resultant forces/moments
    * to get strain, multiply forces and moments by stiffness matrix
    * you need to either translate forces to the bending axis or translate stiffness matrix to the lifting line location
    * GXBeam- see papers for theory
    * VABS- commercial code does Precomp and CompositeStructures
    * CompositeStructures- least finished, but eventually should take beam deflections to actual wing element deflections

* structures parameters -> optional parameters can be set here; translation from VortexLattice objects available
* freestream -> new structure translated to VortexLattice, CCBlade
* Atmosphere.jl properties -> new structure translated to VortexLattice, CCBlade
* airfoil.jl or something... AirfoilParams package... incomplete but helps to parameterize; CST parameters for airfoil optimization
* geometry engine- base on CompositeStructures
* solutions: dictionary defined by user or template
* user inputs:
* observers/algorithms/operations: -> list of functions; default templates available
* templates: load modules and set solutions, etc.

# Files

<!-- * `preroutines/` # processes required to prepare a simulation -->
* `system/`

    * `common.jl`

        * simple geometry definition defined for the package
        * to be translated into geometries for use by other packages
        * likely also contains additional elements:

            * lifting surfaces
            * structures
            * rotors
            * batteries
            * motors

    * `compositestructures.jl`

        * translates `common.jl` objects to `compositestructures.jl` objects and vice versa

    * `vortexlattice.jl`

        * translates `common.jl` objects to `vortexlattice.jl` objects and vice versa

    * `ccblade.jl`

        * translates `common.jl` objects to `vortexlattice.jl` objects and vice versa
        * given a rotor geometry and operating conditions, create airfoil polars from Xfoil and compile into a vector of CCBlade airfoil objects; plot airfoils for checking
        * given an airfoil name and Reynolds number, re-run a single polar
        * given a geometry, builds a CCBlade rotor object
        * other translation functions from `geometry.jl` definitions to `CCBlade`

* `action/`

    * `action.jl`

        <!-- * `abstract type Interaction end` -->
        * `struct Action`
        * instances contain a function to be run during each iteration of a simulation and additional parameters
        * other files in this directory contain functionals that return constructors for `Action` structs based on simulation data

    * `rotor_on_wing.jl`

        * `Interaction` object and function

    * `wing_on_rotor.jl`

        * `Interaction` object and function

    * `aero_on_structures.jl`

        * `Interaction` object and function

* `observers/`

    * `observer.jl`

        * abstract type defining observers
        * functions that accept a system object and a solution dictionary and update the solution

* `templates/`

    * contains `template` abstract type (essentially a list of `Interaction` and `observer` structs) and files with convenience `template` objects for common applications
    * `template.jl`
    * `blownwing.jl`

* `simulation/`

    * `simulation.jl`
    * function run every time
    * time series analysis or not
    * eventually, incorporate unsteady_analysis in VortexLattice?
    * writes VTK files
    * iterates over actions
    * returns solution dictionary populated by `Observer` objects

* `vehicles/`

    * convenience functions to build common system configurations, eg a single fuselage biplane

# Documentation

Visit the documentation [here](urlgoeshere).
