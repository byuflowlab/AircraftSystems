#=##############################################################################################
Filename: ccblade.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file is meant to provide convenience functions for building CCBlade rotor objects.
        This includes running all necessary airfoil analysis and store files in an accessible
        directory structure.
=###############################################################################################

"""
Defines CCBlade rotor geometry. Contains:

* `rotors::Vector{CCBlade.Rotor}`
* `sectionlists::Vector{Vector{CCBlade.Section}}`
* `rlists::Vector{Vector{Float64}}`
* `index::Vector{Int}`
* `positions::Vector{Vector{Float64}}` : x-y-z position in the body frame for each rotor
* `orientations::Vector{Vector{Float64}}` : unit vectors defining the axis of rotation in the direction of positive thrust for each rotor
* `spindirection::Vector{Bool}` : `true` if the rotation vector is in the same direction as positive thrust (by the right-hand rule)

"""
struct CCBladeSystem{V1,V2,V3,V4,V5,V6,V7}
    rotors::V1
    sectionlists::V2
    rlists::V3
    index::V4
    positions::V5
    orientations::V6
    spindirections::V7
end

"""
Convenience constructor for `CCBladeSystem`.

Inputs:

* `nblades_list::Vector{Int64}`
* `rhub_list::Vector{Float64}`
* `rtip_list::Vector{Float64}`
* `radii_list::Vector{Vector{Float64}}`
* `chords_list::Vector{Vector{Float64}}`
* `twists_list::Vector{Vector{Float64}}`
* `airfoilcontours_list::Vector{Vector{String}}`
* `airfoilnames_list::Vector{Vector{String}}`
* `index::Vector{Int64}`
* `positions::Vector{Vector{Float64}}`
* `orientations::Vector{Vector{Float64}}`
* `spindirection::Vector{Bool}`

Default Arguments:

* `Res_list = [fill([5e4, 1e5, 1e6], length(radii_list[i])) for i in 1:length(nblades_list)]` : vector of vectors describes Reynold's numbers at which to run Xfoil for each rotor section

Optional Arguments:

* `machcorrection` : CCBlade Mach correction object
* `skipstart = 1` : number of header lines in airfoil contour file
* `xfoil_alpha = range(-20.0, stop=20.0, length= 161)` : angles of attack at which to run Xfoil
* `M = 0` : Mach number (WARNING: REQUIRES FURTHER DEVELOPMENT FOR NONZERO MACH NUMBER)
* `ν = 1.5e-5` : kinematic viscosity of the freestream
* `Re_digits = -4` : number of digits to round Reynolds number
* `runxfoil=true` : toggle running Xfoil
* `xfoil_iter = 300` : max iterations per angle of attack for Xfoil
* `xfoil_npan = 200` : number of panels for Xfoil to interpolate
* `xfoil_clmaxstop = true` : automatically detect cl_max and stop Xfoil sweep
* `xfoil_clminstop = true` : automatically detect cl_min and stop Xfoil sweep
* `viternaextrapolation=true` : apply the viterna extrapolation
* `rotationcorrection=true` : apply rotational correction
* `rotationcorrection_J = 2.0` : advance ratio at which to apply rotational correction
* `radians = false` : units of written files
* `savefiles = true` : save airfoil files to be read later
* `useoldfiles = true` : read old files if available
* `polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY)` : save directory for airfoil files
* `plotoutput = true` : plot airfoil data; quickly show which polars may need to be modified
* `saveplots = true` : save airfoil plots
* `plotextension = ".pdf"` : file extension for saved airfoil plots
* `verbose = true` : verbose output
* `v_lvl = 0` : number of indentations in verbose output

Outputs:

* `rotorsystem::CCBladeSystem` : constructs a `CCBladeSystem` struct

"""
function CCBladeSystem(nblades_list, rhub_list, rtip_list, radii_list, chords_list, twists_list, airfoilcontours_list::Vector{Vector{String}}, airfoilnames_list, index, positions, orientations, spindirections,
    Res_list = [fill([5e4, 1e5, 1e6], length(radii_list[i])) for i in 1:length(nblades_list)];
    kwargs...
)
    # check input sizes
    for (iorientation, orientation) in enumerate(orientations)
        @assert isapprox(LA.norm(orientation), 1.0) "`orientations` must be unit vectors; check the $(iorientation)th vector"
    end
    # build CCBlade.Rotor objects
    rotors = buildrotors(nblades_list, radii_list, rhub_list, rtip_list; machcorrection = CC.PrandtlGlauert(), kwargs...)
    # build CCBlade.Section lists
    sectionlists = buildsections(nblades_list, rtip_list, radii_list, chords_list, twists_list, Res_list, airfoilcontours_list, airfoilnames_list; kwargs...)
    # build rlists
    rlists = sections2radii.(sectionlists)
    # check index length
    @assert length(index) == length(positions) "length of `index` must match the length of `positions`"
    @assert length(index) == length(orientations) "length of `index` must match the length of `orientations`"
    @assert maximum(index) <= length(rotors) "maximum rotor index cannot exceed the number of rotors"
    # build rotor system
    rotorsystem = CCBladeSystem(rotors, sectionlists, rlists, index, positions, orientations, spindirections)

    return rotorsystem
end

"""
Multiple dispatch for multiple Mach numbers.
"""
function CCBladeSystem(nblades_list, rhub_list, rtip_list, radii_list, chords_list, twists_list, airfoilcontours_list::Vector{Vector{String}}, airfoilnames_list, index, positions, orientations, spindirections, Res_list, Ms_list;
    kwargs...#, closefigure = true
)
    # check input sizes
    for (iorientation, orientation) in enumerate(orientations)
        @assert isapprox(LA.norm(orientation), 1.0) "`orientations` must be unit vectors; check the $(iorientation)th vector"
    end
    # build CCBlade.Rotor objects
    rotors = buildrotors(nblades_list, radii_list, rhub_list, rtip_list; kwargs...)
    # build CCBlade.Section lists
    sectionlists = buildsections(nblades_list, rtip_list, radii_list, chords_list, twists_list, Res_list, Ms_list, airfoilcontours_list, airfoilnames_list; kwargs...)
    # build rlists
    rlists = sections2radii.(sectionlists)
    # check index length
    @assert length(index) == length(positions) "length of `index` must match the length of `positions`"
    @assert length(index) == length(orientations) "length of `index` must match the length of `orientations`"
    @assert maximum(index) <= length(rotors) "maximum rotor index cannot exceed the number of rotors"
    # build rotor system
    rotorsystem = CCBladeSystem(rotors, sectionlists, rlists, index, positions, orientations, spindirections)

    return rotorsystem
end

"""
Multiple dispatch for pre-created airfoil object.
"""
function CCBladeSystem(nblades_list, rhub_list, rtip_list, radii_list, chords_list, twists_list, airfoilfunctions, index, positions, orientations, spindirections;
    kwargs...#, closefigure = true
)
    # check input sizes
    for (iorientation, orientation) in enumerate(orientations)
        @assert isapprox(LA.norm(orientation), 1.0) "`orientations` must be unit vectors; check the $(iorientation)th vector"
    end
    # build CCBlade.Rotor objects
    rotors = buildrotors(nblades_list, radii_list, rhub_list, rtip_list; kwargs...)
    # build CCBlade.Section lists
    sectionlists = buildsections(nblades_list, rtip_list, radii_list, chords_list, twists_list, airfoilfunctions; kwargs...)
    # build rlists
    rlists = sections2radii.(sectionlists)
    # check index length
    @assert length(index) == length(positions) "length of `index` must match the length of `positions`"
    @assert length(index) == length(orientations) "length of `index` must match the length of `orientations`"
    @assert maximum(index) <= length(rotors) "maximum rotor index cannot exceed the number of rotors"
    # build rotor system
    rotorsystem = CCBladeSystem(rotors, sectionlists, rlists, index, positions, orientations, spindirections)

    return rotorsystem
end

@inline function buildrotors(nblades_list, radii_list, rhub_list, rtip_list; machcorrection = nothing, kwargs...)
    @assert length(nblades_list) == length(radii_list) "list of nblades and list of radii must be the same length"
    @assert length(nblades_list) == length(rhub_list) "list of nblades and list of hub radii must be the same length"
    @assert length(nblades_list) == length(rtip_list) "list of nblades and list of tip radii must be the same length"
    rotors = Vector{CC.Rotor}(undef,length(nblades_list))
    for i in 1:length(rotors)
        # note: Mach correction on the fly not required
        rotors[i] = CC.Rotor(rhub_list[i], rtip_list[i], nblades_list[i]; precone=0.0, turbine=false, mach=machcorrection, re=nothing, rotation=nothing, tip=CC.PrandtlTipHub()) #mach=CC.PrandtlGlauert()
    end
    return rotors
end

@inline function buildsections(nblades_list, rtip_list, radii_list, chords_list, twists_list, airfoilfunctions; kwargs...)
    @assert length(nblades_list) == length(chords_list) "list of nblades and list of chords must be the same length"
    @assert length(nblades_list) == length(twists_list) "list of nblades and list of twists must be the same length"
    @assert length(nblades_list) == length(airfoilfunctions) "list of nblades and list of airfoilfunctions must be the same length"
    sectionlists = rotor2sections.(rtip_list, radii_list, chords_list, twists_list, airfoilfunctions; kwargs...)
    return sectionlists
end

@inline function buildsections(nblades_list, rtip_list, radii_list, chords_list, twists_list, Res_list, Ms_list, airfoilcontours_list, airfoilnames_list; kwargs...)
    @assert length(nblades_list) == length(chords_list) "list of nblades and list of chords must be the same length"
    @assert length(nblades_list) == length(twists_list) "list of nblades and list of twists must be the same length"
    @assert length(nblades_list) == length(airfoilcontours_list) "list of nblades and list of airfoil contour files must be the same length"
    @assert length(nblades_list) == length(airfoilnames_list) "list of nblades and list of airfoil names must be the same length"
    # sectionlists = Vector{Vector{CC.Section}}(undef,length(rotors))
    sectionlists = rotor2sections.(rtip_list, radii_list, chords_list, twists_list, Res_list, Ms_list, airfoilcontours_list, airfoilnames_list; kwargs...)
    return sectionlists
end

@inline function buildsections(nblades_list, rtip_list, radii_list, chords_list, twists_list, Res_list, airfoilcontours_list, airfoilnames_list; kwargs...)
    @assert length(nblades_list) == length(chords_list) "list of nblades and list of chords must be the same length"
    @assert length(nblades_list) == length(twists_list) "list of nblades and list of twists must be the same length"
    @assert length(nblades_list) == length(airfoilcontours_list) "list of nblades and list of airfoil contour files must be the same length"
    @assert length(nblades_list) == length(airfoilnames_list) "list of nblades and list of airfoil names must be the same length"
    # sectionlists = Vector{Vector{CC.Section}}(undef,length(rotors))
    sectionlists = rotor2sections.(rtip_list, radii_list, chords_list, twists_list, Res_list, airfoilcontours_list, airfoilnames_list; kwargs...)
    return sectionlists
end

"""
Inputs:

* `rtip::Float64`: rotor tip radius
* `radii::Vector{Float64}`: list of radial stations in absolute units
* `chords::Vector{Float64}`: list of chord lengths at each radial station
* `twists::Vector{Float64}`: list of twist angles in radians at each radial station
* `Res::Vector{Vector{Float64}}`: [i][:]th element is a list of Reynolds numbers at the ith radial station
* `airfoilcontours::Vector{String}`: list of paths to
* `airfoilnames::Vector{String}`:
"""
function rotor2sections(rtip, radii, chords, twists, Res, airfoilcontours, airfoilnames; kwargs...)
    cr75 = FM.linear(radii ./ rtip, chords, 0.75) / rtip
    polars = rotor2polars(radii, chords, cr75, Res, airfoilcontours, airfoilnames; kwargs...)
    sections = CC.Section.(radii, chords, twists, polars)
    return sections
end

function rotor2sections(rtip, radii, chords, twists, Res, Ms, airfoilcontours, airfoilnames; kwargs...)
    cr75 = FM.linear(radii ./ rtip, chords, 0.75) / rtip
    polars = rotor2polars(radii, chords, cr75, Res, Ms, airfoilcontours, airfoilnames; kwargs...)
    sections = CC.Section.(radii, chords, twists, polars)
    return sections
end

"""
Inputs:


"""
function rotor2sections(rtip, radii, chords, twists, airfoilfunctions; kwargs...)
    cr75 = FM.linear(radii ./ rtip, chords, 0.75) / rtip
    sections = CC.Section.(radii, chords, twists, airfoilfunctions)
    return sections
end

function sections2radii(sections)
    return [section.r for section in sections]
end

"""
Rather than provide Reynolds numbers for each section, provide performance metrics and estimate the Reynold's number.

Additional arguments:

* `vinf_low::Float64` : lowest expected freestream velocity
* `vinf_high::Float64` : highest expected freestream velocity
* `rpm_low::Float64` : lowest expected RPM
* `rpm_high::Float64` : highest expected RPM
* `N_Re_list::Vector{Vector{Int}}` : each element is a vector whose elements prescribe how many Reynold's numbers to use at each radial station to interpolate airfoil data
"""
function CCBladeSystem(nblades_list, rhub_list, rtip_list, radii_list, chords_list, twists_list, airfoilcontours_list, airfoilnames_list, index, positions, orientations, spindirection,
        vinf_low, vinf_high, rpm_low, rpm_high, N_Re_list;
        kwargs...
    )
    Res_list = Re_range.(radii_list, chords_list, Ref(vinf_low), Ref(vinf_high), Ref(rpm_low), Ref(rpm_high), N_Re_list; kwargs...)

    return CCBladeSystem(nblades_list, rhub_list, rtip_list, radii_list, chords_list, twists_list, airfoilcontours_list, airfoilnames_list, index, positions, orientations, spindirection, Res_list; kwargs...)
end

"""
Build a vector of airfoil functions for each rotor section.

Inputs:

* `radii::Vector{Float64}` : radii of each rotor section in absolute units
* `chords::Vector{Float64}` : chord of each rotor section in absolute units
* `cr75::Float64` : local chord to tip radius at 75% r/R location
* `Res_list::Vector{Vector{Int64}}`
* `contourfiles::Vector{String}` : vector of paths to .dat airfoil contour file
* `airfoilnames::Vector{String}` : vector of names of each airfoil

Optional Arguments:

* `skipstart = 1` : number of header lines in contour file
* `xfoil_alpha = range(-20.0, stop=20.0, length= 161)` : angles of attack at which to run Xfoil
* `M = 0` : Mach number
* `ν = NU` : kinematic viscocity
* `Re_digits = -4` : number of digits to round the Reynolds number
* `runxfoil = true` : write uncorrected polars from Xfoil
* `xfoil_iter = 300` : max iterations for Xfoil
* `xfoil_npan = 200` : number of panels for Xfoil
* `xfoil_clmaxstop = true` : stop alpha sweep when cl reaches a maximum
* `xfoil_clminstop = true` : stop alpha sweep when cl reaches a minimum
* `radians = false` : desired units for polar files
* `savefiles = true` : save final polars to file for future reference
* `useoldfiles = true` : use old polar files (if they exist)
* `polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY)` : path to save polars and plots
* `plotoutput = true` : toggle ploting polars
* `saveplots = true` : toggle saving plots
* `plotextension = ".pdf"` : extension of saved plots
* `verbose = true` : toggle verbose output
* `v_lvl = 0` : set verbosity level

Outputs:

* vector of `CCBlade.AlphaReAF` objects
* if `savefiles == true`, writes files storing airfoil data for quick reference later
* if `saveplots == true`, saves plots to `polardirectory` for reference

"""
function rotor2polars(radii, chords, cr75, Res_list, contourfiles, airfoilnames;
    polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY), runxfoil = true, useoldfiles = true,
    viternaextrapolation=true, rotationcorrection=true, radians = false, kwargs...)

    @assert length(radii) == length(chords) "length of radii and chords inconsistent"
    @assert length(radii) == length(Res_list) "length of radii ($(length(radii))) and Res_list ($(length(Res_list))) inconsistent"
    @assert length(radii) == length(contourfiles) "length of radii and contourfiles inconsistent"
    @assert length(radii) == length(airfoilnames) "length of radii and airfoilnames inconsistent"

    airfoilobjects = Array{CC.AlphaReAF,1}(undef,length(radii))
    # get polars for each radial station
    for (i_radius, radius) in enumerate(radii)
        Res = Res_list[i_radius]
        # check if corrected files exist
        if useoldfiles
            filenames_corrected = airfoilfilenames(airfoilnames[i_radius], Res; M=0, viternaextrapolation=viternaextrapolation, rotationcorrection=rotationcorrection, aoaset=false)
            filepaths_corrected = joinpath.(Ref(polardirectory), filenames_corrected)
            i_existingfiles_corrected = isfile.(filepaths_corrected)
            if prod(i_existingfiles_corrected)
                airfoilobjects[i_radius] = CC.AlphaReAF(filepaths_corrected; radians = radians)
                continue
            end
        end
        # check if uncorrected files already exist
        filenames_uncorrected = airfoilfilenames(airfoilnames[i_radius], Res; M=0, viternaextrapolation=false, rotationcorrection=false, aoaset=false)
        filepaths_uncorrected = joinpath.(Ref(polardirectory), filenames_uncorrected)
        i_existingfiles_uncorrected = isfile.(filepaths_uncorrected)
        if runxfoil && (!useoldfiles || !prod(i_existingfiles_uncorrected))
            airfoil2xfoil(Res, contourfiles[i_radius], airfoilnames[i_radius]; runxfoil = runxfoil, useoldfiles = useoldfiles, polardirectory = polardirectory, kwargs...)
        end
        airfoilobjects[i_radius] = correctalignpolars(Res, airfoilnames[i_radius], cr75; polardirectory = polardirectory, viternaextrapolation = viternaextrapolation, rotationcorrection = rotationcorrection, kwargs...)
    end
    return airfoilobjects
end

"""
Multiple dispatch for providing Mach corrections a priori.
"""
function rotor2polars(radii, chords, cr75, Res_list, Ms_list, contourfiles, airfoilnames;
    polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY), runxfoil = true, useoldfiles = true,
    viternaextrapolation=true, rotationcorrection=true, radians = false, kwargs...)

    @assert length(radii) == length(chords) "length of radii and chords inconsistent"
    @assert length(radii) == length(Res_list) "length of radii ($(length(radii))) and Res_list ($(length(Res_list))) inconsistent"
    @assert length(radii) == length(contourfiles) "length of radii and contourfiles inconsistent"
    @assert length(radii) == length(airfoilnames) "length of radii and airfoilnames inconsistent"

    airfoilobjects = Array{CC.AlphaReMachAF,1}(undef,length(radii))
    # get polars for each radial station
    for (i_radius, radius) in enumerate(radii)
        Res = Res_list[i_radius]
        Ms = Ms_list[i_radius]
        # check if corrected files exist
        if useoldfiles
            filenames_corrected = airfoilfilenames(airfoilnames[i_radius], Res, Ms; viternaextrapolation=viternaextrapolation, rotationcorrection=rotationcorrection, aoaset=false)
            filepaths_corrected = joinpath.(Ref(polardirectory), filenames_corrected)
            i_existingfiles_corrected = isfile.(filepaths_corrected)
            if prod(i_existingfiles_corrected)
                airfoilobjects[i_radius] = CC.AlphaReMachAF(filepaths_corrected; radians = radians)
                continue
            end
        end
        # check if uncorrected files already exist
        filenames_uncorrected = airfoilfilenames(airfoilnames[i_radius], Res, Ms; viternaextrapolation=false, rotationcorrection=false, aoaset=false)
        filepaths_uncorrected = joinpath.(Ref(polardirectory), filenames_uncorrected)
        i_existingfiles_uncorrected = isfile.(filepaths_uncorrected)
        if runxfoil && (!useoldfiles || !prod(i_existingfiles_uncorrected))
            airfoil2xfoil(Res, Ms, contourfiles[i_radius], airfoilnames[i_radius]; runxfoil = runxfoil, useoldfiles = useoldfiles, polardirectory = polardirectory, kwargs...)
        end
        airfoilobjects[i_radius] = correctalignpolars(Res, Ms, airfoilnames[i_radius], cr75; polardirectory = polardirectory, viternaextrapolation = viternaextrapolation, rotationcorrection = rotationcorrection, kwargs...)
    end
    return airfoilobjects
end

"""
Calculate Reynolds numbers based on the expected performance envelope.

Inputs:

* radii::Vector - radii of each rotor section in absolute units
* chords::Vector - chord of each rotor section in absolute units
* cr75::Float - local chord to tip radius at 75% r/R location
* vinf_low::Float - lowest expected freestream velocity
* vinf_high::Float - highest expected freestream velocity
* rpm_low::Float - lowest expected RPM value
* rpm_high::Float - highest expected RPM value
* N_Re::Vector - vector of number of Reynold's numbers desired for interpolation
* contourfiles::Vector{String} - vector of paths to .dat airfoil contour file
* airfoilnames::Vector{String} - vector of names of each airfoil

"""
function rotor2polars(radii, chords, cr75, vinf_low, vinf_high, rpm_low, rpm_high, N_Re, contourfiles, airfoilnames; kwargs...)

    Res_list = Re_range.(radii, chords, Ref(vinf_low), Ref(vinf_high), Ref(rpm_low), Ref(rpm_high), N_Re; kwargs...)

    return rotor2polars(radii, chords, cr75, Res_list, contourfiles, airfoilnames; kwargs...)
end

"""
Creates uncorrected polar files from an airfoil contour file at designated Reynolds numbers.

Inputs:

* `Res::Vector` : vector of Reynolds numbers at which to run Xfoil
* `contourfile::String` : path to airfoil contour file
* `airfoilname::String` : name of airfoil

Optional Inputs:

* `skipstart = 1` : number of header lines in contour file
* `xfoil_alpha = range(-20.0, stop=20.0, length= 161)` : angles of attack at which to run Xfoil
* `M = 0` : Mach number
* `ν = NU` : kinematic viscocity
* `Re_digits = -4` : number of digits to round the Reynolds number
* `xfoil_iter = 300` : max iterations for Xfoil
* `xfoil_npan = 200` : number of panels for Xfoil
* `xfoil_clmaxstop = true` : stop alpha sweep when cl reaches a maximum
* `xfoil_clminstop = true` : stop alpha sweep when cl reaches a minimum
* `radians = false` : desired units for polar files
* `useoldfiles = true` : use old polar files (if they exist)
* `polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY)` : path to save polars and plots
* `plotoutput = true` : toggle ploting polars
* `saveplots = true` : toggle saving plots
* `plotextension = ".pdf"` : extension of saved plots
* `verbose = true` : toggle verbose output
* `v_lvl = 0` : set verbosity level

Outputs:

* writes polar files in `polardirectory`
* saves plots to `polardirectory`
* returns `nothing`

"""
function airfoil2xfoil(Res, contourfile, airfoilname;
        skipstart = 1, # 1 line header in .dat contour file
        xfoil_alpha = range(-25.0, stop=25.0, length= 161), # every quarter degree
        M = 0, ν = 1.5e-5, Re_digits = -4,
        xfoil_iter = 300, xfoil_npan = 200, xfoil_clmaxstop = true, xfoil_clminstop = true,
        radians = false, useoldfiles = true, polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        plotoutput = true, saveplots = true, plotextension = ".pdf", closefigure = true,
        verbose = true, v_lvl = 0, kwargs...
    )
    # if verbose; println("\t"^v_lvl, "Preparing $contourfile:\n","\t"^v_lvl,"------------------------------------\n"); end
    @assert isfile(contourfile) "`contourfile` must be a path to a csv formatted airfoil contour file"
    # bookkeeping for filenames
    filenames_uncorrected = airfoilfilenames(airfoilname, Res; M=0, viternaextrapolation=false, rotationcorrection=false, aoaset=false)
    filepaths_uncorrected = joinpath.(Ref(polardirectory), filenames_uncorrected)
    i_existingfiles_uncorrected = isfile.(filepaths_uncorrected)
    # prepare polar directory
    if !isdir(polardirectory); mkpath(polardirectory); end
    # write vanilla polars from xfoil
    if verbose; println("\t"^v_lvl, "Reading $contourfile\n"); end
    xy = DF.readdlm(contourfile, ',', Float64, '\n'; skipstart = skipstart)
    for (i_Re, Re) in enumerate(Res)
        if verbose; println("\n\t"^v_lvl, "==================== Re = $Re ===================="); end
        if !i_existingfiles_uncorrected[i_Re] || !useoldfiles
            # write uncorrected polar
            cl, cd, cdp, cm, conv = XF.alpha_sweep(xy[:,1], xy[:,2], xfoil_alpha, Re;
                    iter=xfoil_iter,
                    npan=xfoil_npan,
                    mach=M,
                    percussive_maintenance=true,
                    printdata=verbose,
                    zeroinit=true,
                    clmaxstop=xfoil_clmaxstop,
                    clminstop=xfoil_clminstop
                    )
            # create object and save files
            angleconversion = radians ? pi / 180.0 : 1
            thisobject = CC.AlphaAF(xfoil_alpha[conv] .* angleconversion, cl[conv], cd[conv], airfoilname, Re * 1.0, M * 1.0)
            CC.write_af(filepaths_uncorrected[i_Re], thisobject)
            if plotoutput
                plotairfoil(thisobject, filenames_uncorrected[i_Re], airfoilname, Re, M;
                    viternaextrapolation = false, rotationcorrection = false,
                    savefigure = saveplots, savepath = polardirectory,
                    extension = plotextension, closefigure = closefigure)
            end
        end
    end

    return nothing
end

"""
Multiple dispatch for multiple Mach numbers.
"""
function airfoil2xfoil(Res, Ms, contourfile, airfoilname;
        skipstart = 1, # 1 line header in .dat contour file
        xfoil_alpha = range(-20.0, stop=20.0, length= 161), # every quarter degree
        ν = 1.5e-5, Re_digits = -4,
        xfoil_iter = 300, xfoil_npan = 200, xfoil_clmaxstop = true, xfoil_clminstop = true,
        radians = false, useoldfiles = true, polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        plotoutput = true, saveplots = true, plotextension = ".pdf", closefigure = true,
        verbose = true, v_lvl = 0, kwargs...
    )
    # if verbose; println("\t"^v_lvl, "Preparing $contourfile:\n","\t"^v_lvl,"------------------------------------\n"); end
    @assert isfile(contourfile) "`contourfile` must be a path to a csv formatted airfoil contour file"
    # bookkeeping for filenames
    filenames_uncorrected = airfoilfilenames(airfoilname, Res, Ms; viternaextrapolation=false, rotationcorrection=false, aoaset=false)
    filepaths_uncorrected = joinpath.(Ref(polardirectory), filenames_uncorrected)
    i_existingfiles_uncorrected = isfile.(filepaths_uncorrected)
    # prepare polar directory
    if !isdir(polardirectory); mkpath(polardirectory); end
    # write vanilla polars from xfoil
    if verbose; println("\t"^v_lvl, "Reading $contourfile\n"); end
    xy = DF.readdlm(contourfile, ',', Float64, '\n'; skipstart = skipstart)
    for (i_M, M) in enumerate(Ms)
        for (i_Re, Re) in enumerate(Res)
            if verbose; println("\n\t"^v_lvl, "==================== Re = $Re ===================="); end
            if !i_existingfiles_uncorrected[i_Re] || !useoldfiles
                # write uncorrected polar
                cl, cd, cdp, cm, conv = XF.alpha_sweep(xy[:,1], xy[:,2], xfoil_alpha, Re;
                        iter=xfoil_iter,
                        npan=xfoil_npan,
                        mach=M,
                        percussive_maintenance=true,
                        printdata=verbose,
                        zeroinit=true,
                        clmaxstop=xfoil_clmaxstop,
                        clminstop=xfoil_clminstop
                        )
                # create object and save files
                angleconversion = radians ? pi / 180.0 : 1
                thisobject = CC.AlphaAF(xfoil_alpha[conv] .* angleconversion, cl[conv], cd[conv], airfoilname, Re * 1.0, M * 1.0)
                CC.write_af(filepaths_uncorrected[i_Re, i_M], thisobject)
                if plotoutput
                    plotairfoil(thisobject, filenames_uncorrected[i_Re, i_M], airfoilname, Re, M;
                        viternaextrapolation = false, rotationcorrection = false,
                        savefigure = saveplots, savepath = polardirectory,
                        extension = plotextension, closefigure = closefigure)
                end
            end
        end
    end

    return nothing
end

"""
Applies Viterna extrapolation, rotational correction, and aligns angles of attack of existing polar files and builds a `CCBlade.AlphaReAF` object.

Inputs:

* `Res::Vector{Int}` : vector of Reynold's numbers at which to find files
* `airfoilname::String` : name of the airfoil
* `cr75` : 75% r/R chord over tip radius

Optional inputs:

* `M = 0`: Mach number
* `viternaextrapolation = true` : perform the Viterna exprapolation to high angles of attack
* `rotationcorrection = true` : perform a rotational correction to airfoil files
* `rotationcorrection_J = 1.0` : advance ratio at which to perform the rotational correction
* `radians = false` : airfoil file units
* `savefiles = true` : save corrected airfoil to files
* `polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY)` : directory to save polars and plots
* `plotoutput = true` : plot resulting airfoil data
* `saveplots = true` : save plots to file
* `verbose = true` : toggle verbose output
* `v_lvl = 0` : set verbose layer

Outputs:

* `CCBlade.AlphaReAF` object
* if `savefiles == true`, writes files storing airfoil data for quick reference later
* if `plotoutput` and `saveplots == true`, saves plots of the polars

"""
function correctalignpolars(Res, airfoilname, cr75;
        M = 0, viternaextrapolation=true, rotationcorrection=true, rotationcorrection_J = 1.0,
        radians = false, savefiles = true, polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        plotoutput = true, saveplots = true, plotextension = ".pdf", closefigure = true, kwargs...
    )

    # bookkeeping for filenames
    filenames_uncorrected = airfoilfilenames(airfoilname, Res; M=0, viternaextrapolation=false, rotationcorrection=false, aoaset=false)
    filepaths_uncorrected = joinpath.(Ref(polardirectory), filenames_uncorrected)
    i_existingfiles_uncorrected = isfile.(filepaths_uncorrected)

    @assert prod(i_existingfiles_uncorrected) "1 or more uncorrected polar files do not exist; run `airfoil2xfoil` first"

    filenames = airfoilfilenames(airfoilname, Res; M=0, viternaextrapolation=viternaextrapolation, rotationcorrection=rotationcorrection, aoaset=false)
    filepaths = joinpath.(Ref(polardirectory), filenames)

    # filenames_aoaset = airfoilfilenames(airfoilname, Res; M=0, viternaextrapolation=viternaextrapolation, rotationcorrection=rotationcorrection, aoaset=true)
    # filepaths_aoaset = joinpath.(Ref(polardirectory), filenames_aoaset)
    # compile viterna and rotation corrected files
    α_list = []
    cl_list = []
    cd_list = []
    for (i_Re, Re) in enumerate(Res)
        thisobject = CC.AlphaAF(filepaths_uncorrected[i_Re]; radians = radians)
        α = thisobject.alpha
        cl = thisobject.cl
        cd = thisobject.cd
        if viternaextrapolation
            α, cl, cd = CC.viterna(α, cl, cd, cr75) # only use converged values
            if plotoutput
                plotairfoil(α, cl, cd, filenames[i_Re], airfoilname, Re, M;
                    viternaextrapolation = true, rotationcorrection = false,
                    savefigure = saveplots, savepath = polardirectory,
                    extension = plotextension, tag = "", clearfigure = true,
                    closefigure = true
                )
            end
        end
        if rotationcorrection
            for (i_α, this_α) in enumerate(α)
                cl[i_α], cd[i_α] = CC.rotation_correction(CC.DuSeligEggers(), cl[i_α], cd[i_α], cr75, 0.75, pi/rotationcorrection_J, this_α)
            end
            if plotoutput
                plotairfoil(α, cl, cd, filenames[i_Re], airfoilname, Re, M;
                    viternaextrapolation = viternaextrapolation, rotationcorrection = true,
                    savefigure = saveplots, savepath = polardirectory,
                    extension = plotextension, tag = "", clearfigure = false,
                    closefigure = closefigure
                )
            end
        end
        radians ? push!(α_list, α) : push!(α_list, α .* 180/pi)
        push!(cl_list, cl)
        push!(cd_list, cd)
    end
    # interpolate to common alphas
    α_table = α_list[1]
    Re_table = Res
    cl_table = zeros(length(α_table), length(Res))
    cd_table = similar(cl_table)
    for i_Re = 1:length(Re_table)
        cl_table[:,i_Re] .= FM.linear(α_list[i_Re], cl_list[i_Re], α_table)
        cd_table[:,i_Re] .= FM.linear(α_list[i_Re], cd_list[i_Re], α_table)
    end
    # build object
    objecttags = ""
    if viternaextrapolation; objecttags *= ", extrapolated"; end
    if rotationcorrection; objecttags *= ", w/ rotation correction"; end
    airfoilobject = CC.AlphaReAF(α_table, typeof(α_table[1]).(Re_table), cl_table, cd_table, "$airfoilname" * objecttags)
    # save files
    if savefiles
        CC.write_af(filepaths, airfoilobject)
    end

    return airfoilobject
end

"""
Multiple dispatch for multiple Mach numbers.
"""
function correctalignpolars(Res, Ms, airfoilname, cr75;
    viternaextrapolation=true, rotationcorrection=true, rotationcorrection_J = 2.0,
    radians = false, savefiles = true, polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
    plotoutput = true, saveplots = true, plotextension = ".pdf", closefigure = true, kwargs...
)
    # bookkeeping for filenames
    filenames_uncorrected = airfoilfilenames(airfoilname, Res, Ms; viternaextrapolation=false, rotationcorrection=false, aoaset=false)
    filepaths_uncorrected = joinpath.(Ref(polardirectory), filenames_uncorrected)
    i_existingfiles_uncorrected = isfile.(filepaths_uncorrected)

    @assert prod(i_existingfiles_uncorrected) "1 or more uncorrected polar files do not exist; run `airfoil2xfoil` first"

    filenames = airfoilfilenames(airfoilname, Res, Ms; viternaextrapolation=viternaextrapolation, rotationcorrection=rotationcorrection, aoaset=false)
    filepaths = joinpath.(Ref(polardirectory), filenames)

    # filenames_aoaset = airfoilfilenames(airfoilname, Res; M=0, viternaextrapolation=viternaextrapolation, rotationcorrection=rotationcorrection, aoaset=true)
    # filepaths_aoaset = joinpath.(Ref(polardirectory), filenames_aoaset)
    # compile viterna and rotation corrected files
    α_list = Array{Vector,2}(undef, length(Res), length(Ms))
    cl_list = Array{Vector,2}(undef, length(Res), length(Ms))
    cd_list = Array{Vector,2}(undef, length(Res), length(Ms))
    for (i_M, M) in enumerate(Ms)
        for (i_Re, Re) in enumerate(Res)
            thisobject = CC.AlphaAF(filepaths_uncorrected[i_Re, i_M]; radians = radians)
            α = thisobject.alpha
            cl = thisobject.cl
            cd = thisobject.cd
            if viternaextrapolation
                α, cl, cd = CC.viterna(α, cl, cd, cr75) # only use converged values
                if plotoutput
                    plotairfoil(α, cl, cd, filenames[i_Re, i_M], airfoilname, Re, M;
                        viternaextrapolation = true, rotationcorrection = false,
                        savefigure = saveplots, savepath = polardirectory,
                        extension = plotextension, tag = "", clearfigure = true, closefigure = true
                    )
                end
            end
            if rotationcorrection
                for (i_α, this_α) in enumerate(α)
                    cl[i_α], cd[i_α] = CC.rotation_correction(CC.DuSeligEggers(), cl[i_α], cd[i_α], cr75, 0.75, pi/rotationcorrection_J, this_α)
                end
                if plotoutput
                    plotairfoil(α, cl, cd, filenames[i_Re, i_M], airfoilname, Re, M;
                        viternaextrapolation = viternaextrapolation, rotationcorrection = true,
                        savefigure = saveplots, savepath = polardirectory,
                        extension = plotextension, tag = "", clearfigure = false, closefigure = closefigure
                    )
                end
            end
            radians ? α_list[i_Re, i_M] = α : α_list[i_Re, i_M] = α .* 180/pi
            cl_list[i_Re, i_M] = cl
            cd_list[i_Re, i_M] = cd
        end
    end
    # interpolate to common alphas
    α_table = α_list[1,1]
    Re_table = Res
    M_table = Ms
    cl_table = zeros(length(α_table), length(Res), length(Ms))
    cd_table = similar(cl_table)
    for i_M = 1:length(M_table)
        for i_Re = 1:length(Re_table)
            cl_table[:,i_Re, i_M] .= FM.linear(α_list[i_Re, i_M], cl_list[i_Re, i_M], α_table)
            cd_table[:,i_Re, i_M] .= FM.linear(α_list[i_Re, i_M], cd_list[i_Re, i_M], α_table)
        end
    end
    # build object
    objecttags = ""
    if viternaextrapolation; objecttags *= ", extrapolated"; end
    if rotationcorrection; objecttags *= ", w/ rotation correction"; end
    airfoilobject = CC.AlphaReMachAF(α_table, typeof(α_table[1]).(Re_table), typeof(α_table[1]).(M_table), cl_table, cd_table, "$airfoilname" * objecttags)
    # save files
    if savefiles
        CC.write_af(filepaths, airfoilobject)
    end

    return airfoilobject
end

function airfoilfilenames(airfoilname, Res;
        M=0, viternaextrapolation=false, rotationcorrection=false,
        aoaset=false, extension = ".txt"
    )
    tags = ""
    if viternaextrapolation; tags *= "_ext"; end
    if rotationcorrection; tags *= "_rot"; end
    if aoaset; tags *= "_aoaset"; end
    [airfoilname * "_Re$(Int(round(Re; digits=0)))" * tags * extension for Re in Res]
end

function airfoilfilenames(airfoilname, Res, Ms;
        viternaextrapolation=false, rotationcorrection=false,
        aoaset=false, extension = ".txt"
    )
    tags = ""
    if viternaextrapolation; tags *= "_ext"; end
    if rotationcorrection; tags *= "_rot"; end
    if aoaset; tags *= "_aoaset"; end
    [airfoilname * "_Re$(Int(round(Re; digits=0)))" * "_M$(M)" * tags * extension for Re in Res, M in Ms]
end

function Re_range(radius, chord, vinf_low, vinf_high, rpm_low, rpm_high, N_Re; ν = 1.5e-5, Re_digits=-4, kwargs...)
    vtangential_low = rpm_low / 60.0 * 2 * pi * radius
    vtot_low = sqrt(vinf_low^2 + vtangential_low^2)
    Re_low = chord * vtot_low / ν

    vtangential_high = rpm_high / 60.0 * 2 * pi * radius
    vtot_high = sqrt(vinf_high^2 + vtangential_high^2)
    Re_high = chord * vtot_high / ν

    Res = range(Re_low, stop=Re_high, length=N_Re)
    Res_truncated = Int.(round.(collect(Res), RoundUp, digits=Re_digits))
    Res_truncated[1] = round(Res[1], RoundDown, digits=Re_digits)
    unique!(Res_truncated)
    filter!(x -> x > 0, Res_truncated)

    return Res_truncated
end

"""
Plots airfoil polars.

Inputs:

* α::Vector : vector of angles of attack
* cl::Vector : vector of lift coefficents
* cd::Vector : vector of drag coefficents
* filename::String : name of the associated CCBlade airfoil polar file
* airfoilname::String : name of the airfoil
* Re : Reynolds number
* M : Mach number
* viternaextrapolation = false : toggle viterna extrapolation
* rotationcorrection = false : toggle rotation correction
* savefigure = false : save the figure
* savepath = joinpath(topdirectory, "data", "airfoil", "polars", TODAY) : directory to save the figure
* extension = ".pdf" : figure extension
* tag = "" : string to append to the series legend
* clearfigure = true : clear the figure before ploting (set false if you want to compare series)

"""
function plotairfoil(α, cl, cd, filename, airfoilname, Re, M;
        radians = false,
        viternaextrapolation = false, rotationcorrection = false,
        savefigure = true, savepath = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        extension = ".pdf", tag = "", clearfigure = true, closefigure = true
    )
    # set up units
    aoaunits = L" [^\circ]"
    conversionfactor = radians ? 180.0 / pi : 1
    # set up figure
    fig = plt.figure(filename)
    if clearfigure
        fig.clear()
        fig.suptitle(airfoilname * ": Re = $Re, M = $M")
        axes = []
        push!(axes, fig.add_subplot(211))
        axes[1].set_ylabel(L"c_l")
        push!(axes, fig.add_subplot(212))
        axes[2].set_ylabel(L"c_d")
        axes[2].set_xlabel(L"\alpha" * aoaunits)
    else
        axes = fig.get_axes()
    end
    # set labels
    labeltag = ""
    viternaextrapolation ? labeltag *= "Ext." : labeltag *= "Not ext."
    rotationcorrection ? labeltag *= " w/ rot. corr." : labeltag *= " w/o rot. corr."
    labeltag *= tag
    # plot data
    axes[1].plot(α .* conversionfactor, cl, label = labeltag)
    axes[2].plot(α .* conversionfactor, cd, label = labeltag)
    # show figure
    axes[1].legend(loc="upper left", bbox_to_anchor=(1.03,0.9), prop=Dict("size" => 9))
    fig.subplots_adjust(left=0.1, right=0.7, top=0.9, bottom=0.1)
    # fig.show()
    # save plot
    if savefigure
        if !isdir(savepath); mkpath(savepath); end
        savename = splitext(filename)[1] * extension
        fig.savefig(joinpath(savepath, savename))
    end
    if closefigure; plt.close(filename); end

    return nothing
end

"Overload `plotairfoil` to accept a CCBlade.AlphaAF object."
function plotairfoil(airfoilobject::CCBlade.AlphaAF, filename, airfoilname, Re, M; kwargs...)
    plotairfoil(airfoilobject.alpha, airfoilobject.cl, airfoilobject.cd, filename, airfoilname, Re, M; kwargs...)
end

"""
    plotrotor(filename, rotorname, rs, chords, twists)

Plots rotor geometry.

Inputs:

* `filename::String` : used to save the file
* `rotorname::String` : label for the current plot series
* `rs::Vector{Float64}` : radial stations of the rotor
* `chords::Vector{Float64}` : chord lengths at each radial station
* `twists::Vector{Float64}` : twists at each radial station

Keyword Arguments:

* `cratio = 1.0` : value between 0 and 1 used to set the plot color
* `savefigure = true` : save the figure
* `savepath = joinpath(topdirectory, "data", "plots", TODAY)` : directory to save the figure
* `extension = ".pdf",` : file extension to the saved file
* `clearfigure = true` : clear the figure before plotting
* `closefigure = true` : close the figure upon completion

"""
function plotrotor(filename, rotorname, rs, chords, twists;
        cratio = 1.0, savefigure = true, savepath = joinpath(topdirectory, "data", "plots", TODAY), extension = ".pdf",
        clearfigure = true, closefigure = true
    )
    # set up units
    aoaunits = L" [^\circ]"
    conversionfactor = radians ? 180.0 / pi : 1
    # set up figure
    fig = plt.figure(filename)
    if clearfigure
        fig.clear()
        fig.suptitle("Rotor Geometry")
        axes = []
        fig.add_subplot(211)
        fig.add_subplot(212)
        axes = fig.get_axes()
        axes[1].set_ylabel(L"c")
        axes[2].set_ylabel(L"\theta")
        axes[2].set_xlabel(L"\alpha" * aoaunits)
    else
        axes = fig.get_axes()
    end
    # plot data
    axes[1].plot(rs, chords, label = rotorname, color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio))
    axes[2].plot(rs, twists, label = rotorname, color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio))
    # save plot
    if savefigure
        if !isdir(savepath); mkpath(savepath); end
        savename = splitext(filename)[1] * extension
        fig.savefig(joinpath(savepath, savename))
    end
    if closefigure; fig.close(); end

    return nothing
end

"""
Inputs:

* `freestream::Freestream` : freestream object
* `orientation::Vector{Float64}` : unit vector aligned with the rotor axis of rotation in the direction of positive thrust

Outputs:

* `vinflow::Float64` : the magnitude of the freestream in the negative thrust direction.
"""
function rotorinflow(freestream, orientation)#, interstream)
    Vinf = freestream2vector(freestream)
    vinflow = -LA.dot(Vinf, orientation)

    return vinflow
end

"""
Inputs:

* `r::Vector{Float64}` : vector of a rotor's radial sections
* `omega::Float64` : rotational velocity in rad/s
* `orientation::Vector{Float64}` : vector defining the rotor axis of rotation by the right hand rule
* `freestream::Freestream` : `Freestream` struct
* `environment::Environment` : `Environment` struct

Outputs:

* `operatingpoints::Vector{CCBlade.OperatingPoint}` : vector of operating points for a single rotor

"""
function rotor2operatingpoints(vinflow, r, omega, environment)

    operatingpoints = CC.simple_op.(vinflow, omega, r, environment.ρ; pitch = 0.0, mu = environment.μ, asound = environment.a, precone = 0.0)

    return operatingpoints
end

function solverotorsnondimensional(rotorsystem, omegas, freestream, environment)#, interstream::Interstream)
    # extract rotor info
    orientations = rotorsystem.orientations
    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)
    # pre-allocate solution
    Js, Ts, Qs, us, vs = solverotors(rotorsystem, omegas, freestream, environment)
    CTs = similar(Qs)
    CQs = similar(Qs)
    ηs = similar(Qs)
    # iterate over rotors
    for (i, rotorindex) in enumerate(rotorsystem.index)
        # isolate rotor
        rotor = rotorsystem.rotors[rotorindex]
        η, CT, CQ = CC.nondim(Ts[i], Qs[i], vinflows[i], omegas[i], environment.ρ, rotor, "propeller")
        CTs[i] = CT
        CQs[i] = CQ
        ηs[i] = η
    end

    return Js, CTs, CQs, ηs
end

"""
In-place version of solverotorsnondimensional.
"""
function solverotorsnondimensional!(Js, Ts, Qs, CTs, CQs, ηs, rotorsystem, omegas, freestream, environment)#, interstream::Interstream)
    # extract rotor info
    orientations = rotorsystem.orientations
    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)
    # pre-allocate solution
    solverotors!(Js, Ts, Qs, rotorsystem, omegas, freestream, environment)
    # iterate over rotors
    for (i, rotorindex) in enumerate(rotorsystem.index)
        # isolate rotor
        rotor = rotorsystem.rotors[rotorindex]
        η, CT, CQ = CC.nondim(Ts[i], Qs[i], vinflows[i], omegas[i], environment.ρ, rotor, "propeller")
        CTs[i] = CT
        CQs[i] = CQ
        ηs[i] = η
    end

    return nothing
end

"""
In-place version of solverotorsnondimensional including `us` and `vs`.
"""
function solverotorsnondimensional!(Js, Ts, Qs, CTs, CQs, ηs, us, vs, rotorsystem, omegas, freestream, environment)#, interstream::Interstream)
    # extract rotor info
    orientations = rotorsystem.orientations
    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)
    # pre-allocate solution
    solverotors!(Js, Ts, Qs, us, vs, rotorsystem, omegas, freestream, environment)
    # iterate over rotors
    for (i, rotorindex) in enumerate(rotorsystem.index)
        # isolate rotor
        rotor = rotorsystem.rotors[rotorindex]
        η, CT, CQ = CC.nondim(Ts[i], Qs[i], vinflows[i], omegas[i], environment.ρ, rotor, "propeller")
        CTs[i] = CT
        CQs[i] = CQ
        ηs[i] = η
    end

    return nothing
end

"""
    solverotors(rotorsystem, omegas, freestream, environment)

Solves the rotor system at the specified RPM, freestream, and environment.

Inputs:

* `rotorsystem::CCBladeSystem` : a rotor system
* `omegas::Vector{Float64} : vector of rotational velocities in rad/s of each rotor
* `freestream::Freestream` : Freestream object
* `environment::Environment` : Environment object

Outputs:

* `Js::Vector{Float64}` : vector of advance ratios of each rotor
* `Ts::Vector{Float64}` : vector of thrust values of each rotor
* `Qs::Vector{Float64}` : vector of torque values of each rotor
* `us::Vector{Vector{Float64}}` : [i][j] refers to the axial induced velocity at the rotor disk at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}` : [i][j] refers to the swirl induced velocity at the rotor disk at the jth radial station of the ith rotor

"""
function solverotors(rotorsystem, omegas, freestream, environment)#, interstream::Interstream)
    # pre-allocate solution
    Js = zeros(length(rotorsystem.index))
    Ts = similar(Js)
    Qs = similar(Ts)
    us = [zeros(typeof(freestream.vinf), length(radii)) for radii in rotorsystem.rlists[rotorsystem.index]]
    vs = deepcopy(us)

    solverotors!(Js, Ts, Qs, us, vs, rotorsystem, omegas, freestream, environment)

    return Js, Ts, Qs, us, vs
end

"""
In-place instance of `solverotors`.

Inputs:

* `Js::Vector{Float64}` : vector of advance ratios of each rotor
* `Ts::Vector{Float64}` : vector of thrust values of each rotor
* `Qs::Vector{Float64}` : vector of torque values of each rotor
* `us::Vector{Vector{Float64}}` : [i][j] refers to the axial induced velocity at the rotor disk at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}` : [i][j] refers to the swirl induced velocity at the rotor disk at the jth radial station of the ith rotor
* `rotorsystem::CCBladeSystem` : a rotor system
* `omegas::Vector{Float64} : vector of rotational velocities in rad/s of each rotor
* `freestream::Freestream` : Freestream object
* `environment::Environment` : Environment object

Modifies:

* `Js::Vector{Float64}` : vector of advance ratios of each rotor
* `Ts::Vector{Float64}` : vector of thrust values of each rotor
* `Qs::Vector{Float64}` : vector of torque values of each rotor
* `us::Vector{Vector{Float64}}` : [i][j] refers to the axial induced velocity at the rotor disk at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}` : [i][j] refers to the swirl induced velocity at the rotor disk at the jth radial station of the ith rotor

"""
function solverotors!(Js, Ts, Qs, us, vs, rotorsystem, omegas, freestream, environment)#, interstream::Interstream)
    # extract rotor info
    rlists = rotorsystem.rlists[rotorsystem.index]
    orientations = rotorsystem.orientations
    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)
    operatingpoints_list = rotor2operatingpoints.(vinflows, rlists, omegas, Ref(environment))
    # iterate over rotors
    for (i, rotorindex) in enumerate(rotorsystem.index)
        # isolate rotor
        rotor = rotorsystem.rotors[rotorindex]
        sections = rotorsystem.sectionlists[rotorindex]
        # isolate operating conditions
        operatingpoints = operatingpoints_list[i]
        # solve CCBlade
        outputs = CC.solve.(Ref(rotor), sections, operatingpoints)
        T, Q = CC.thrusttorque(rotor, sections, outputs)
        Ts[i] = T
        Qs[i] = Q
        us[i][:] .= outputs.u
        vs[i][:] .= outputs.v
    end
    # calculate Js
    Js .= vinflows ./ omegas * 2 * pi ./ [2 * rotor.Rtip for rotor in rotorsystem.rotors[rotorsystem.index]]

    return nothing
end

"""
In-place instance of `solverotors`.

Inputs:

* `Js::Vector{Float64}` : vector of advance ratios of each rotor
* `Ts::Vector{Float64}` : vector of thrust values of each rotor
* `Qs::Vector{Float64}` : vector of torque values of each rotor
* `us::Vector{Vector{Float64}}` : [i][j] refers to the axial induced velocity at the rotor disk at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}` : [i][j] refers to the swirl induced velocity at the rotor disk at the jth radial station of the ith rotor
* `rotorsystem::CCBladeSystem` : a rotor system
* `omegas::Vector{Float64} : vector of rotational velocities in rad/s of each rotor
* `freestream::Freestream` : Freestream object
* `environment::Environment` : Environment object

Modifies:

* `Js::Vector{Float64}` : vector of advance ratios of each rotor
* `Ts::Vector{Float64}` : vector of thrust values of each rotor
* `Qs::Vector{Float64}` : vector of torque values of each rotor
* `us::Vector{Vector{Float64}}` : [i][j] refers to the axial induced velocity at the rotor disk at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}` : [i][j] refers to the swirl induced velocity at the rotor disk at the jth radial station of the ith rotor

"""
function solverotors!(Js, Ts, Qs, rotorsystem, omegas, freestream, environment)#, interstream::Interstream)
    # extract rotor info
    rlists = rotorsystem.rlists[rotorsystem.index]
    orientations = rotorsystem.orientations
    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)
    operatingpoints_list = rotor2operatingpoints.(vinflows, rlists, omegas, Ref(environment))
    # iterate over rotors
    for (i, rotorindex) in enumerate(rotorsystem.index)
        # isolate rotor
        rotor = rotorsystem.rotors[rotorindex]
        sections = rotorsystem.sectionlists[rotorindex]
        # isolate operating conditions
        operatingpoints = operatingpoints_list[i]
        # solve CCBlade
        outputs = CC.solve.(Ref(rotor), sections, operatingpoints)
        T, Q = CC.thrusttorque(rotor, sections, outputs)
        Ts[i] = T
        Qs[i] = Q
    end
    # calculate Js
    Js .= vinflows ./ omegas * 2 * pi ./ [2 * rotor.Rtip for rotor in rotorsystem.rotors[rotorsystem.index]]

    return Js, Ts, Qs
end

solverotor(rotor, sections, operatingpoints) = CC.solve.(Ref(rotor), sections, operatingpoints)

function solverotors_outputs(rotorsystem, omegas, freestream, environment)
    # extract rotor info
    rlists = rotorsystem.rlists[rotorsystem.index]
    orientations = rotorsystem.orientations
    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)
    operatingpoints_list = rotor2operatingpoints.(vinflows, rlists, omegas, Ref(environment))
    # solve rotors
    outputs_list = solverotor.(rotorsystem.rotors[rotorsystem.index], rotorsystem.sectionlists[rotorsystem.index], operatingpoints_list)
    # calculate Js
    Js = vinflows ./ omegas * 2 * pi ./ [2 * rotor.Rtip for rotor in rotorsystem.rotors[rotorsystem.index]]

    return Js, outputs
end

"""
    induced2wakefunction(rotorsystem, us, vs, xD_fullydeveloped_u, xD_fullydeveloped_v)

Interpolates rotor-induced velocities into a wake function of X.

Inputs:

* `rotorsystem::CCBladeSystem` : system of rotors
* `us::Vector{Vector{Float64}}` : [i][j]th element is the axial induced velocity at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}` : [i][j]th element is the swirl induced velocity at the jth radial station of the ith rotor

Keyword Arguments:

* `wakeshapefunction = (Rtip, x) -> Rtip,` : function accepts a rotor radius and axial coordinate and returns the local wake radius
* `axialinterpolation = (rs, us, r) -> FM.linear(rs, us, r)` : function accepts radial stations, axial velocities, and the radial coordinate and returns the unmodified axial induced velocity
* `swirlinterpolation = (rs, vs, r) -> FM.linear(rs, vs, r)` : function accepts radial stations, swirl velocities, and the radial coordinate and returns the unmodified swirl induced velocity
* `axialmultiplier = (u, distance2plane, Rtip) -> u * 2` : function multiplied by the axial interpolation function
* `swirlmultiplier = (v, distance2plane, Rtip) -> v` : function multiplied by the swirlinterpolation function

"""
function induced2wakefunction(rotorsystem, us, vs;
        wakeshapefunction = (Rtip, x) -> Rtip,
        axialinterpolation = (rs, us, r) -> FM.linear(rs, us, r),
        swirlinterpolation = (rs, vs, r) -> FM.linear(rs, vs, r),
        axialmultiplier = (distance2plane, Rtip) -> 2,
        swirlmultiplier = (distance2plane, Rtip) -> 1
    )
    function wakefunction(X)
        Vwake = zeros(3)
        for (irotor, rotorindex) in enumerate(rotorsystem.index)
            # translate origin to rotor center (R is the vector from the rotor center to X)
            R = X - rotorsystem.positions[irotor]
            r = LA.norm(R)
            # calculate distance from rotor plane (positive is in the negative thrust direction)
            distance2plane = -LA.dot(R, rotorsystem.orientations[irotor])
            # angle between normal and R (\in [0,\pi])
            θ = acos(distance2plane/r)
            # calculate distance to centerline
            distance2centerline = r * sin(θ) # always positive
            # check if X is within the wake
            Rtip = rotorsystem.rotors[rotorindex].Rtip
            if distance2centerline < wakeshapefunction(Rtip, distance2plane)
                # interpolate wake velocity
                u = axialinterpolation(rotorsystem.rlists[rotorindex], us[irotor], distance2centerline) * axialmultiplier(distance2plane, Rtip)
                v = swirlinterpolation(rotorsystem.rlists[rotorindex], vs[irotor], distance2centerline) * swirlmultiplier(distance2plane, Rtip)
                # get axial unit vector
                axialhat = -rotorsystem.orientations[irotor]
                # get tangential unit vector
                tangentialhat = LA.cross(rotorsystem.orientations[irotor], R)
                tangentialhat ./= LA.norm(tangentialhat)
                # project R onto the plane parallel to normal
                Vinduced = u * axialhat + v * tangentialhat * (rotorsystem.spindirections[irotor] ? 1 : -1)
                Vwake .+= Vinduced
            end
        end
        return Vwake
    end
end
