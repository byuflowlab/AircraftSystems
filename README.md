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

# Documentation

Visit the documentation [here](urlgoeshere).
