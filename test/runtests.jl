import AircraftSystems
AS = AircraftSystems
FM = AS.FM
plt = AS.plt
using Test

# topdirectory = normpath(joinpath(@__DIR__,".."))

# @testset "rotor.jl" begin
    # radius = 0.3
    # chord = 0.03
    # cr75 = 0.1
    # vinf_low = 1.0
    # vinf_high = 10.0
    # rpm_low = 1000
    # rpm_high = 5000
    # N_Re = 5
    airfoilname = "naca4412"
    airfoilfile = joinpath(AS.topdirectory, "data", "airfoil", "contours", "naca4412.dat")
    # airfoilobject = AS.airfoil2object(radius, chord, cr75, vinf_low, vinf_high, rpm_low, rpm_high, N_Re, airfoilfile, airfoilname;
    #     skipstart = 1,
    #     xfoil_alpha = range(-20.0, stop=20.0, length= 161), # every quarter degree
    #     ν = AS.NU, Re_digits = -4,
    #     runxfoil=false, xfoil_iter = 300, xfoil_npan = 200,
    #     viternaextrapolation=true, rotationcorrection=true,
    #     radians = false, savefiles = true, useoldfiles = false, polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", AS.TODAY),
    #     plotoutput = true, saveplots = true, plotextension = ".pdf",
    #     verbose = true, v_lvl = 0
    # )

    radii = [0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-2/2
    twists = [35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0] * pi/180
    chords = [9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] * 1e-3
    # cr75 = FM.akima(radii ./ 236e-3 * 2, chords ./ radii, 0.75)
    vinf_low = 35.0
    vinf_high = 50.0
    rpm_low = 5000
    rpm_high = 10000
    N_Re = fill(3, length(radii))
    airfoilnames = fill(airfoilname, length(radii))
    contourfiles = fill(airfoilfile, length(radii))
    # airfoilobjects = AS.rotor2airfoil(radii, chords, cr75,
    #         vinf_low, vinf_high, rpm_low, rpm_high, N_Re,
    #         contourfiles, airfoilnames;
    #         skipstart = 1,
    #         xfoil_alpha = range(-20.0, stop=20.0, length= 161), # every quarter degree
    #         M = 0, ν = AS.NU, Re_digits = -4,
    #         runxfoil=true, xfoil_iter = 300, xfoil_npan = 200,
    #         xfoil_clmaxstop = true, xfoil_clminstop = true,
    #         viternaextrapolation=true, rotationcorrection=true, rotationcorrection_J = 2.0,
    #         radians = false, savefiles = true, useoldfiles = true, polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", AS.TODAY),
    #         plotoutput = true, saveplots = true, plotextension = ".pdf",
    #         verbose = true, v_lvl = 0
    #     )
    nblades_list = [2]
    rhub_list = [radii[1]]
    rtip_list = [radii[end]]
    radii_list = [radii]
    chords_list = [chords]
    twists_list = [twists]
    airfoilcontours_list = [contourfiles]
    airfoilnames_list = [airfoilnames]
    index = [1,1]
    positions = fill((0.0,0.0,0.0),2)
    orientations = fill((-1.0,0.0,0.0),2)
    ccbladesystem = AS.CCBladeSystem(
        nblades_list,
        rhub_list,
        rtip_list,
        radii_list,
        chords_list,
        twists_list,
        airfoilcontours_list,
        airfoilnames_list,
        index,
        positions,
        orientations
    )

    omegas = fill(5000, length(ccbladesystem.index))
    freestream = AS.Freestream(20.0, 15.0 * pi/180, 3.0 * pi/180, [0.0, 0.0, 0.0])
    environment = AS.Environment()
    ops_list = AS.CC.OperatingPoint(ccbladesystem, omegas, freestream, environment)
# end
