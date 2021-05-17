# AircraftSystems

Author: Ryan Anderson
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
* structures -> defined in GXBeam or PreComp or CompositeStructures
* structures parameters -> optional parameters can be set here; translation from VortexLattice objects available
* freestream -> new structure translated to VortexLattice, CCBlade
* atmospheric properties -> new structure translated to VortexLattice, CCBlade
* solutions: dictionary defined by user or template
* user inputs:
* observers/algorithms/operations: -> list of functions; default templates available
* templates: load modules and set solutions, etc.

# Documentation

Visit the documentation [here](urlgoeshere).
