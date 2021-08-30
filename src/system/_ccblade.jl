#=##############################################################################################
Filename: ccblade.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file is meant to provide convenience functions for building CCBlade rotor objects.
        This includes running all necessary airfoil analysis and store files in an accessible
        directory structure.
=###############################################################################################


"""
    CCBladeSystem{V1,V2,V3,V4,V5,V6,V7}

Defines CCBlade rotor geometry.

# Fields:

* `rotors::Vector{CCBlade.Rotor}`
* `sectionlists::Vector{Vector{CCBlade.Section}}`
* `rlists::Vector{Vector{Float64}}`
* `index::Vector{Int}`
* `positions::Vector{Vector{Float64}}` : x-y-z position in the body frame for each rotor
* `orientations::Vector{Vector{Float64}}` : unit vectors defining the axis of rotation in the direction of positive thrust for each rotor
* `spindirection::Vector{Bool}` : `true` if the rotation vector is in the same direction as positive thrust (by the right-hand rule)

"""
struct CCBladeSystem{TF,TF1,TF2,TF3,TAF}
        # T1 <: Union{Nothing, CC.Mach_correction}, T2 <: Union{Nothing, CC.ReCorrection},
        # T3 <: Union{Nothing, CC.RotationCorrection}, T4 <: Union{Nothing, CC.TipCorrection}}
    rotors::Vector{CC.Rotor}#{TF, Int64, Bool, T1, T2, T3, T4}}
    sectionlists::Vector{Vector{CCBlade.Section{TF1, TF2, TF3, TAF}}}
    rlists::Vector{Vector{TF}}
    index::Vector{Int64}
    positions::Vector{Vector{TF}}
    orientations::Vector{Vector{TF}}
    spin_directions::Vector{Bool}
end

# AircraftSystems.CCBladeSystem(
#     ::Array{CCBlade.Rotor,1},
#     ::Array{Array{CCBlade.Section{Float64,Float64,Float64,CCBlade.AlphaReAF{Float64,String}},1},1},
#     ::Array{Array{Float64,1},1},
#     ::Array{Int64,1},
#     ::Array{Array{Float64,1},1},
#     ::Array{Array{Float64,1},1},
#     ::Array{Bool,1})

"""
    CCBladeSystem(nblades_list, rhub_list, rtip_list, radii_list, chords_list, twists_list,
        airfoilcontours_list, airfoilnames_list, index, positions, orientations, spin_directions,
        Res_list = [fill([5e4, 1e5, 1e6], length(radii_list[i])) for i in 1:length(nblades_list)];
        kwargs...)

Convenience constructor for `CCBladeSystem`.

# Arguments:

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

# Keyword Arguments:

* `Res_list`: vector of vectors describes Reynold's numbers at which to run Xfoil for each rotor section


# Optional Arguments:

* `mach_correction` : CCBlade Mach correction object
* `skipstart = 1`: number of header lines in airfoil contour file
* `xfoil_alpha = range(-20.0, stop=20.0, length= 161)`: angles of attack at which to run Xfoil
* `M = 0`: Mach number (WARNING: REQUIRES FURTHER DEVELOPMENT FOR NONZERO MACH NUMBER)
* `ν = 1.5e-5`: kinematic viscosity of the freestream
* `Re_digits = -4`: number of digits to round Reynolds number
* `runxfoil=true`: toggle running Xfoil
* `xfoil_iter = 300`: max iterations per angle of attack for Xfoil
* `xfoil_npan = 200`: number of panels for Xfoil to interpolate
* `xfoil_clmaxstop = true`: automatically detect cl_max and stop Xfoil sweep
* `xfoil_clminstop = true`: automatically detect cl_min and stop Xfoil sweep
* `viternaextrapolation=true`: apply the viterna extrapolation
* `rotationcorrection=true`: apply rotational correction
* `rotationcorrection_J = 2.0`: advance ratio at which to apply rotational correction
* `radians = false`: units of written files
* `savefiles = true`: save airfoil files to be read later
* `useoldfiles = true`: read old files if available
* `polar_directory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY)`: save directory for airfoil files
* `plotoutput = true`: plot airfoil data; quickly show which polars may need to be modified
* `saveplots = true`: save airfoil plots
* `plot_extension = ".pdf"`: file extension for saved airfoil plots
* `verbose = true`: verbose output
* `v_lvl = 0`: number of indentations in verbose output

# Returns:

* `rotor_system::CCBladeSystem`: constructs a `CCBladeSystem` struct

"""
function CCBladeSystem(rotors, sectionlists, index, positions, orientations, spin_directions)

    # check input sizes
    for (iorientation, orientation) in enumerate(orientations)
        @assert isapprox(LA.norm(orientation), 1.0) "`orientations` must be unit vectors; check the $(iorientation)th vector"
    end

    @assert length(index) == length(positions) "length of `index` must match the length of `positions`"
    @assert length(index) == length(orientations) "length of `index` must match the length of `orientations`"
    @assert maximum(index) <= length(rotors) "maximum rotor index cannot exceed the number of rotors"

    rlists = sections2radii.(sectionlists)

    return CCBladeSystem(rotors, sectionlists, rlists, index, positions, orientations, spin_directions)
end

function CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
    sectionlists, rlists, index, positions, orientations, spin_directions; kwargs...
)
    # build CCBlade.Rotor objects
    rotors = build_rotors(nblades_list, rhub_list, rtip_list, mach_correction; kwargs...)

    return CCBladeSystem(rotors, sectionlists, rlists, index, positions, orientations, spin_directions)
end

function CCBladeSystem(rotors,
    radii_list, chords_list, twists_list, ccblade_af_objects_list,
    index, positions, orientations, spin_directions
)
    # build CCBlade.Section lists
    sectionlists = build_sections(radii_list, chords_list, twists_list, ccblade_af_objects_list)

    return CCBladeSystem(rotors, sectionlists, index, positions, orientations, spin_directions)
end

function CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
    radii_list, chords_list, twists_list, ccblade_af_objects_list,
    index, positions, orientations, spin_directions;
        kwargs...
)
    # build CCBlade.Rotor objects
    rotors = build_rotors(nblades_list, rhub_list, rtip_list, mach_correction; kwargs...)

    return CCBladeSystem(rotors,
        radii_list, chords_list, twists_list, ccblade_af_objects_list,
        index, positions, orientations, spin_directions
    )
end

function CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
    radii_list, chords_list, twists_list,
    Res_lists, Ms_lists, contour_paths_list, uncorrected_polar_paths_lists, corrected_polar_paths_lists,
    index, positions, orientations, spin_directions;
        kwargs...
)
    # build af objects
    ccblade_af_objects_list = generate_polars.(rtip_list, radii_list, chords_list, twists_list, Res_lists, Ms_lists, contour_paths_list, uncorrected_polar_paths_lists, corrected_polar_paths_lists; kwargs...)

    return CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
        radii_list, chords_list, twists_list, ccblade_af_objects_list,
        index, positions, orientations, spin_directions
    )
end

function CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
    radii_list, chords_list, twists_list,
    Res_lists, Ms_lists, contour_paths_list::Vector{String},
    index, positions, orientations, spin_directions;
        polar_directory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        kwargs...
)

    uncorrected_polar_paths_lists = Vector{Array{String,2}}(undef, length(nblades_list))
    corrected_polar_paths_lists = Vector{Array{String,2}}(undef, length(nblades_list))
    for i_rotor in 1:length(nblades_list)
        polar_base_names = joinpath(polar_directory, splitext(splitpath(contour_paths_list[i_rotor])[end])[1])
        uncorrected_polar_paths_lists[i_rotor] = airfoil_2_polar_names(polar_base_names, Res_lists[i_rotor], Ms_lists[i_rotor];
            viterna_extrapolation=false, rotation_correction=false,
            extension=".txt"
        )
        corrected_polar_paths_lists[i_rotor] = airfoil_2_polar_names(polar_base_names, Res_lists[i_rotor], Ms_lists[i_rotor];
            viterna_extrapolation=true, rotation_correction=true,
            extension=".txt"
        )
    end

    return CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
        radii_list, chords_list, twists_list,
        Res_lists, Ms_lists, contour_paths_list, uncorrected_polar_paths_lists, corrected_polar_paths_lists,
        index, positions, orientations, spin_directions;
            kwargs...
    )
end

function CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
    radii_list, chords_list, twists_list,
    Res_lists, Ms_lists, contour_paths_list,
    index, positions, orientations, spin_directions;
        polar_directory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        kwargs...
)

    uncorrected_polar_paths_lists = Vector{Vector{Array{String,2}}}(undef, length(nblades_list))
    corrected_polar_paths_lists = Vector{Vector{Array{String,2}}}(undef, length(nblades_list))
    for i_rotor in 1:length(nblades_list)
        polar_base_names = joinpath.(Ref(polar_directory), [splitext(splitpath(polar_i)[end])[1] for polar_i in contour_paths_list[i_rotor]])
        uncorrected_polar_paths_lists[i_rotor] = airfoil_2_polar_names(polar_base_names, Res_lists[i_rotor], Ms_lists[i_rotor];
            viterna_extrapolation=false, rotation_correction=false,
            extension=".txt"
        )
        corrected_polar_paths_lists[i_rotor] = airfoil_2_polar_names(polar_base_names, Res_lists[i_rotor], Ms_lists[i_rotor];
            viterna_extrapolation=true, rotation_correction=true,
            extension=".txt"
        )
    end

    return CCBladeSystem(nblades_list, rhub_list, rtip_list, mach_correction,
        radii_list, chords_list, twists_list,
        Res_lists, Ms_lists, contour_paths_list, uncorrected_polar_paths_lists, corrected_polar_paths_lists,
        index, positions, orientations, spin_directions;
            kwargs...
    )
end

"""
    sections2radii(sections)

# Arguments

* `sections`

# Returns

"""
@inline function sections2radii(sections)
    return [section.r for section in sections]
end

@inline function build_rotors(nblades_list, rhub_list, rtip_list, mach_correction = nothing; kwargs...)

    @assert length(nblades_list) == length(rhub_list) "list of nblades and list of hub radii must be the same length"
    @assert length(nblades_list) == length(rtip_list) "list of nblades and list of tip radii must be the same length"

    rotors = Vector{CC.Rotor}(undef,length(nblades_list))
    for i in 1:length(rotors)
        # note: Mach correction on the fly not required
        # rhub_list[i] =
        rotors[i] = CC.Rotor(rhub_list[i], rtip_list[i], nblades_list[i]; precone=0.0, turbine=false, mach=mach_correction, re=nothing, rotation=nothing, tip=CC.PrandtlTipHub()) #mach=CC.PrandtlGlauert()
    end
    return rotors
end

@inline function build_sections(radii_list, chords_list, twists_list, ccblade_af_objects_list)

    @assert length(chords_list) == length(twists_list) "chords_list and twists must be the same length"
    @assert length(chords_list) == length(ccblade_af_objects_list) "chords_list and ccblade_af_objects_list must be the same length"

    return rotor_2_sections.(radii_list, chords_list, twists_list, ccblade_af_objects_list)
end

@inline function rotor_2_sections(radii, chords, twists, ccblade_af_objects)
    sections = CC.Section.(radii, chords, twists, ccblade_af_objects)
end

"""
    generate_polars(rtip, radii, chords, twists, Res_list, Ms_list,
        contour_paths, uncorrected_polar_paths_list, corrected_polar_paths_list;
            kwargs...
    )

Builds airfoil objects for each radial section of a rotor.

# Arguments

* rtip::Float64
* radii::Vector{Float64}
* chords::Vector{Float64}
* twists::Vector{Float64}
* Res_list::Vector{Vector{Float64}}
* Ms_list::Vector{Vector{Float64}}
* contour_paths::Vector{String}
* uncorrected_polar_paths_list::Vector{Array{String,2}}
* corrected_polar_paths_list::Vector{Array{String,2}}

"""
function generate_polars(rtip, radii, chords, twists, Res_list, Ms_list, contour_paths, uncorrected_polar_paths_list, corrected_polar_paths_list; kwargs...)
    cr75 = FM.linear(radii ./ rtip, chords, 0.75) / FM.linear(radii ./ rtip, radii, 0.75)

    polars = rotor_2_polars(radii, chords, cr75, Res_list, Ms_list, contour_paths, uncorrected_polar_paths_list, corrected_polar_paths_list;
        use_old_files=true, viterna_extrapolation=true, rotation_correction=true, radians_polar=false,
        plot_polars=true, save_figure=true, extension=".png", close_figure=true,
        kwargs...
    )

    return polars
end

"""
    rotor_2_polars(radii, chords, cr75, Res_list, Ms_list, contour_paths, uncorrected_polar_paths_list, corrected_polar_paths_list;
        use_old_files=true, viterna_extrapolation=true, rotation_correction=true, radians_polar=false,
        plot_polars=true, save_figure=true, extension=".png", close_figure=true,
        kwargs...
    )

Build a vector of airfoil functions for each rotor section.

# Arguments:

* `radii::Vector{Float64}` : radii of each rotor section in absolute units
* `chords::Vector{Float64}` : chord of each rotor section in absolute units
* `cr75::Float64` : local chord to tip radius at 75% r/R location
* `Res_list::Vector{Vector{Int64}}`
* `Ms_list::Vector{Vector{Int64}}`
* `contour_paths::Vector{String}` : vector of paths to .dat airfoil contour file
* `uncorrected_polar_paths_list::Vector{Array{String,2}}` : [i][j,k] is the path to the uncorrected polar file of the ith radial station at the jth Reynold's number, kth Mach number
* `corrected_polar_paths_list::Vector{Array{String,2}}` : [i][j,k] is the path to the corrected polar file of the ith radial station at the jth Reynold's number, kth Mach number

# Keyword Arguments:

* `use_old_files=true` : re-use existing polar files
* `viterna_extrapolation=true` : perform the Viterna extrapolation on airfoil data
* `rotation_correction=true` : perform a rotation correction to airfoil data
* `radians_polar=false` : `true` indicates aoa's of polars is in radians
* `plot_polars=true` : plot polars
* `save_figure=true` : save polar plots
* `extension=".png"` : extension of polar plots
* `close_figure=true` : close polar plots after generation

# Returns:

* `airfoil_objects`: vector of `<:CCBlade.AFType` objects

"""
function rotor_2_polars(radii, chords, cr75, Res, Ms, contour_paths::String, uncorrected_polar_paths, corrected_polar_paths;
    use_old_files=true, use_old_corrected_files = true, viterna_extrapolation=true, rotation_correction=true, radians_polar=false,
    plot_polars=true, save_figure=true, extension=".png", close_figure=true,
    kwargs...
)

    @assert length(radii) == length(chords) "length of radii and chords inconsistent"
    @assert length(uncorrected_polar_paths) == length(Res) * length(Ms) "size of uncorrected_polar_paths is $(size(uncorrected_polar_paths)); expected $((length(Res), length(Ms)))"
    @assert length(corrected_polar_paths) == length(Res) * length(Ms) "size of corrected_polar_paths is $(size(corrected_polar_paths)); expected $((length(Res), length(Ms)))"

    # TODO: re-write this to use multiple dispatch rather than if statements
    if length(Res) == 1
        if length(Ms) == 1
            af_type = CC.AlphaAF
        else
            af_type = CC.AlphaMachAF
        end
    else
        if length(Ms) == 1
            af_type = CC.AlphaReAF
        else
            af_type = CC.AlphaReMachAF
        end
    end

    # check if corrected files exist
    # corrected_polar_paths = add_tags.(uncorrected_polar_paths; viterna_extrapolation, rotation_correction)
    i_existing_files_corrected = isfile.(corrected_polar_paths)
    if !(use_old_files && prod(i_existing_files_corrected) && use_old_corrected_files) # if notall files exist
        # write files
        airfoil_2_files(Res, Ms, contour_paths, uncorrected_polar_paths, corrected_polar_paths;
            radians_polar, cr75, use_old_files, use_old_corrected_files,
            plot_polars, save_figure, extension, close_figure,
            viterna_extrapolation, rotation_correction,
            kwargs...
        )
    end

    airfoil_objects = fill(Res_Ms_2_polar(Res, Ms, af_type, corrected_polar_paths, radians_polar), length(radii))

    return airfoil_objects
end

function rotor_2_polars(radii, chords, cr75, Res_list, Ms_list, contour_paths, uncorrected_polar_paths_list, corrected_polar_paths_list;
    use_old_files=true, use_old_corrected_files = true, viterna_extrapolation=true, rotation_correction=true, radians_polar=false,
    plot_polars=true, save_figure=true, extension=".png", close_figure=true,
    kwargs...
)

    @assert length(radii) == length(chords) "length of radii and chords inconsistent"
    @assert length(radii) == length(Res_list) "length of radii ($(length(radii))) and Res_list ($(length(Res_list))) inconsistent"
    @assert length(radii) == length(contour_paths) "length of radii and contour_paths inconsistent"
    @assert length(radii) == length(uncorrected_polar_paths_list) "length of radii and uncorrected_polar_files_list inconsistent"
    @assert size(uncorrected_polar_paths_list[1]) == (length(Res_list[1]), length(Ms_list[1])) "size of uncorrected_polar_files_list[1] is $(size(uncorrected_polar_files[1])); expected $((length(Res_list[1]), length(Ms_list[1])))"
    @assert size(corrected_polar_paths_list[1]) == (length(Res_list[1]), length(Ms_list[1])) "size of corrected_polar_files_list[1] is $(size(corrected_polar_files[1])); expected $((length(Res_list[1]), length(Ms_list[1])))"

    # TODO: re-write this to use multiple dispatch rather than if statements
    if length(Res_list[1]) == 1
        if length(Ms_list[1]) == 1
            af_type = CC.AlphaAF
        else
            af_type = CC.AlphaMachAF
        end
    else
        if length(Ms_list[1]) == 1
            af_type = CC.AlphaReAF
        else
            af_type = CC.AlphaReMachAF
        end
    end

    if !(typeof(contour_paths[1]) <: AbstractArray) && prod(contour_paths .== contour_paths[1]) && # single airfoil
        prod([isequal(Res_list[1], Res_list[i]) for i in 2:length(Res_list)]) && # same Res
        prod([isequal(Ms_list[1], Ms_list[i]) for i in 2:length(Ms_list)]) # same Ms

        airfoil_objects = fill(Res_Ms_2_polar(Res_list[1], Ms_list[1], af_type, corrected_polar_paths_list[1], radians_polar))
    else
        airfoil_objects = Array{af_type,1}(undef,length(radii))

        # get polars for each radial station
        for (i_radius, radius) in enumerate(radii)
            Res = Res_list[i_radius]
            Ms = Ms_list[i_radius]
            uncorrected_polar_paths = uncorrected_polar_paths_list[i_radius]
            corrected_polar_paths = corrected_polar_paths_list[i_radius]

            # check if corrected files exist
            # corrected_polar_paths = add_tags.(uncorrected_polar_paths; viterna_extrapolation, rotation_correction)
            i_existing_files_corrected = isfile.(corrected_polar_paths)
            if use_old_files && prod(i_existing_files_corrected) && use_old_corrected_files # if all files exist
                airfoil_objects[i_radius] = Res_Ms_2_polar(Res, Ms, af_type, corrected_polar_paths, radians_polar)
            else
                # write files
                airfoil_2_files(Res, Ms, contour_paths[i_radius], uncorrected_polar_paths, corrected_polar_paths;
                    radians_polar, cr75, use_old_files, use_old_corrected_files,
                    plot_polars, save_figure, extension, close_figure,
                    viterna_extrapolation, rotation_correction,
                    kwargs...
                )

                # build and store objects
                airfoil_objects[i_radius] = Res_Ms_2_polar(Res, Ms, af_type, corrected_polar_paths, radians_polar)
            end
        end
    end

    return airfoil_objects
end

function Res_Ms_2_polar(Res, Ms, af_type, corrected_polar_paths, radians_polar)
    if length(Res) == 1 && length(Ms) == 1
        return af_type(corrected_polar_paths[1]; radians = radians_polar)
    elseif length(Res) == 1 || length(Ms) == 1
        return af_type(corrected_polar_paths[:]; radians = radians_polar)
    else
        return af_type(corrected_polar_paths; radians = radians_polar)
    end
end

"""
    airfoil_2_files(Res, Ms, contour_path, uncorrected_polar_paths, corrected_polar_paths;
        viterna_extrapolation=true, rotation_correction=true,
        skipstart=1, # 1 line header in .dat contour file
        iter=300, npan=200, clmaxstop=true, clminstop=true,
        xfoil_alpha = range(-20.0, stop=20.0, length= 161), radians_xfoil = false, radians_polar = true,
        cr75 = 0.0,
        use_old_files=true,
        plot_polars=true, save_figure=true, extension=".pdf", close_figure=true,
        verbose=true, v_lvl=0,
        kwargs...
    )

Writes uncorrected and corrected polar files from an airfoil contour file at designated Reynolds and Mach numbers.

# Arguments:

* `Res::Vector` : vector of Reynolds numbers at which to run Xfoil
* `Ms::Vector` : vector of Mach numbers at which to run Xfoil
* `contour_path::String` : path to airfoil contour file
* `uncorrected_polar_paths::Array{String,2}` : [i,j]th element is the path to the uncorrected polar of the ith Reynold's number, jth Mach number
* `corrected_polar_paths::Array{String,2}` : [i,j]th element is the path to the corrected polar of the ith Reynold's number, jth Mach number

# Keyword Arguments:

* `skipstart = 1` : number of header lines in contour file
* `xfoil_alpha = range(-20.0, stop=20.0, length= 161)` : angles of attack at which to run Xfoil
* `M = 0` : Mach number
* `ν = NU` : kinematic viscocity
* `Re_digits = -4` : number of digits to round the Reynolds number
* `xfoil_iter = 300` : max iterations for Xfoil
* `xfoil_npan = 200` : number of panels for Xfoil
* `xfoil_clmaxstop = true` : stop alpha sweep when cl reaches a maximum
* `xfoil_clminstop = true` : stop alpha sweep when cl reaches a minimum
* `radians_xfoil = false` : units of aoa's input to xfoil (`xfoil_alpha`)
* `radians_polar = false` : desired units for polar files
* `cr75 = 0.0` : r/R=0.75 local chord over local radius (used for rotation correction)
* `use_old_files = true` : use old polar files (if they exist)
* `plot_polars = true` : toggle ploting polars
* `save_figure = true` : toggle saving plots
* `extension = ".pdf"` : extension of saved plots
* `close_figure = true` : close the figure after generation
* `verbose = true` : toggle verbose output
* `v_lvl = 0` : set verbosity level

# Returns:

* nothing

"""
function airfoil_2_files(Res, Ms, contour_path, uncorrected_polar_paths, corrected_polar_paths;
    viterna_extrapolation=true, rotation_correction=true,
    skipstart=1, # 1 line header in .dat contour file
    iter=300, npan=200, clmaxstop=true, clminstop=true,
    xfoil_alpha = range(-20.0, stop=20.0, length= 161), radians_xfoil = false, radians_polar = true,
    cr75 = 0.0, rotation_correction_J = 2.0,
    use_old_files=true, use_old_corrected_files = true,
    plot_polars=true, save_figure=true, extension=".pdf", close_figure=true,
    verbose=true, v_lvl=0, kwargs...
)

    @assert isfile(contour_path) "`contour_path` $(contour_path) does not exist"
    @assert length(uncorrected_polar_paths) == length(Res) * length(Ms) "size of uncorrected_polar_paths is $(size(uncorrected_polar_paths)); expected ($(length(Res)), $(length(Ms)))"
    @assert length(corrected_polar_paths) == length(Res) * length(Ms) "size of corrected_polar_paths is $(size(corrected_polar_paths)); expected ($(length(Res)), $(length(Ms)))"

    if verbose; println("\t"^v_lvl, "Reading $contour_path\n"); end
    xy = DF.readdlm(contour_path, ',', Float64, '\n'; skipstart = skipstart)
    percussive_maintenance = true
    α_ref = nothing
    for (i_M, M) in enumerate(Ms)
        for (i_Re, Re) in enumerate(Res)
            if !isfile(uncorrected_polar_paths[i_Re, i_M]) || !use_old_files # if the file already exists
                α, cl, cd = xfoil_2_file(uncorrected_polar_paths[i_Re, i_M], xy[:,1], xy[:,2], Re, M;
                    xfoil_alpha, radians_xfoil, radians_polar,
                    iter, npan, percussive_maintenance, verbose, clmaxstop, clminstop,
                )

                if plot_polars
                    plot_airfoil(α, cl, cd, uncorrected_polar_paths[i_Re, i_M];
                        radians=radians_polar,
                        viterna_extrapolation=false, rotation_correction=false,
                        save_figure, extension, clear_figure = true, close_figure
                    )
                    plot_airfoil(α, cl, cd, corrected_polar_paths[i_Re, i_M];
                        radians=radians_polar,
                        viterna_extrapolation=false, rotation_correction=false,
                        save_figure = save_figure && !viterna_extrapolation && !rotation_correction,
                        extension, clear_figure = true, close_figure = false
                    )
                end
            else # read the file
                polar = CC.AlphaAF(uncorrected_polar_paths[i_Re, i_M]; radians = radians_polar)
                α = polar.alpha
                cl = polar.cl
                cd = polar.cd
            end
            if !isfile(corrected_polar_paths[i_Re, i_M]) || !use_old_corrected_files # if the file already exists, skip this
                convert_to_radians = radians_polar ? 1.0 : pi / 180.0
                if viterna_extrapolation
                    α, cl, cd = CC.viterna(α * convert_to_radians, cl, cd, cr75) # only use converged values
                    α .*= 180/pi
                    if plot_polars
                        plot_airfoil(α, cl, cd, corrected_polar_paths[i_Re, i_M];
                        radians = radians_polar,
                        viterna_extrapolation=true, rotation_correction=false,
                        save_figure = save_figure && !rotation_correction,
                        extension, clear_figure = false, close_figure = false
                        )
                    end
                end

                if rotation_correction
                    for (i_α, this_α) in enumerate(α)
                        # rotation_correction(rc::RotationCorrection, cl, cd, cr, rR, tsr, alpha, phi=alpha, alpha_max_corr=30*pi/180)
                        cl[i_α], cd[i_α] = CC.rotation_correction(CC.DuSeligEggers(), cl[i_α], cd[i_α], cr75, 0.75, pi/rotation_correction_J, this_α * convert_to_radians)
                    end
                    if plot_polars
                        plot_airfoil(α, cl, cd, corrected_polar_paths[i_Re, i_M];
                        radians = radians_polar,
                        viterna_extrapolation, rotation_correction=true,
                        save_figure, extension, clear_figure = false, close_figure = false
                        )
                    end
                end

                # interpolate to consistent aoas
                if i_M == i_Re && i_M == 1; global α_ref = deepcopy(α); end
                cl_aligned = FM.linear(α, cl, α_ref)
                cd_aligned = FM.linear(α, cd, α_ref)
                if plot_polars
                    plot_airfoil(α_ref, cl_aligned, cd_aligned, corrected_polar_paths[i_Re, i_M];
                    radians = radians_polar,
                    viterna_extrapolation, rotation_correction, aligned = true,
                    save_figure, extension, clear_figure = false, close_figure
                    )
                end

                corrected_polar = CC.AlphaAF(α_ref, cl_aligned, cd_aligned, splitpath(corrected_polar_paths[i_Re, i_M])[end], Re, M) # TODO: maybe Re * 1.0, mach * 1.0
                CC.write_af(corrected_polar_paths[i_Re, i_M], corrected_polar)
            end
        end
    end

    return nothing
end

"""

# Arguments

* `radians_xfoil::Bool` : true if the aoas provided are in radians
* `radians_polar::Bool` : true if aoas of the polar object should be in radians

"""
function xfoil_2_file(polar_path, x, y, Re, mach;
    xfoil_alpha = range(-20.0, stop=20.0, length= 161), radians_xfoil = false, radians_polar = true,
    iter=300, npan=200, percussive_maintenance=true, verbose=true, clmaxstop=true, clminstop=true,
)
    if verbose; println("\n", "==================== Re = $Re, M = $mach ===================="); end

    # write uncorrected polar
    angle_conversion_xfoil = radians_xfoil ? 180.0 / pi : 1
    cl_raw, cd_raw, cdp, cm, conv = XF.alpha_sweep(x, y, xfoil_alpha .* angle_conversion_xfoil, Re;
                                iter,
                                npan,
                                mach,
                                percussive_maintenance,
                                printdata=verbose,
                                zeroinit=true,
                                clmaxstop,
                                clminstop)

    # create object and save files
    angle_conversion_polar = radians_polar ? pi/180 : 1.0
    α = xfoil_alpha[conv] .* angle_conversion_polar
    cl = cl_raw[conv]
    cd = cd_raw[conv]
    polar = CC.AlphaAF(α, cl, cd, splitpath(polar_path)[end], Re, mach) # TODO: maybe Re * 1.0, mach * 1.0
    if verbose; println("Writing to $polar_path"); end
    println("pathcheck = $(splitpath(polar_path)[1])")
    if !isdir(splitpath(polar_path)[1]); mkpath(splitpath(polar_path)[1]); end
    CC.write_af(polar_path, polar)

    return α, cl, cd
end

"""
    airfoil_2_polar_names(airfoilname, Res, Ms;
        M=0, viternaextrapolation=false, rotationcorrection=false,
        extension=".txt")

Convenience function to create polar file names.

# Arguments:

* `airfoilname::String`
* `Res`: Reynolds numbers
* `Ms`: Mach numbers

# Keyword Arguments:

* `viternaextrapolation::Bool`
* `rotationcorrection::Bool`
* `aoaset::Bool`
* `extension::String`

"""
function airfoil_2_polar_names(airfoil_name::String, Res, Ms;
            viterna_extrapolation=false, rotation_correction=false,
            extension=".txt")

    tags = ""
    if viterna_extrapolation; tags *= "_ext"; end
    if rotation_correction; tags *= "_rot"; end
    [airfoil_name * "_Re$(Int(round(Re; digits=0)))" * "_M$(M)" * tags * extension for Re in Res, M in Ms]
end

"Multiple dispatch for a vector of airfoil_names."
function airfoil_2_polar_names(airfoil_names, Res, Ms;
    viterna_extrapolation=false, rotation_correction=false,
    extension=".txt")
    [airfoil_2_polar_names(airfoil_name, Res[i], Ms[i];
        viterna_extrapolation, rotation_correction, extension) for (i, airfoil_name) in enumerate(airfoil_names)]
end

# """
# Multiple dispatch for M=0.
# """
# function airfoil_2_polar_names(airfoilname, Res;
#             viterna_extrapolation=false, rotation_correction=false,
#             extension=".txt")

#     tags = ""
#     if viterna_extrapolation; tags *= "_ext"; end
#     if rotation_correction; tags *= "_rot"; end

#     [airfoilname * "_Re$(Int(round(Re; digits=0)))" * tags * extension for Re in Res, M in Ms]
# end

function add_tags(path, tags=""; viterna_extrapolation = false, rotation_correction = false)
    file, ext = splitext(path)
    if viterna_extrapolation; tags *= "_ext"; end
    if rotation_correction; tags *= "_rot"; end

    return file * tags * ext
end

"""
    plot_airfoil(α, cl, cd, filename, airfoilname, Re, M;
        radians=false,
        viternaextrapolation=false, rotationcorrection=false,
        savefigure=true, savepath=joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        extension=".pdf", tag="", clearfigure=true, closefigure=true)

Plots airfoil polars.

# Arguments:

* `α::Vector`: vector of angles of attack
* `cl::Vector`: vector of lift coefficents
* `cd::Vector`: vector of drag coefficents
* `filename::String`: name of the associated CCBlade airfoil polar file
* `airfoilname::String`: name of the airfoil
* `Re`: Reynolds number
* `M`: Mach number

# Keyword Arguments:

* `radians::Bool`
* `viternaextrapolation::Bool`: toggle viterna extrapolation
* `rotationcorrection::Bool`: toggle rotation correction
* `savefigure::Bool`: save the figure
* `savepath::String`: directory to save the figure
* `extension::String`: figure extension
* `tag::String`: string to append to the series legend
* `clearfigure::Bool`: clear the figure before plotting (set false if you want to compare series)
* `closefigure::Bool`:

"""
function plot_airfoil(α, cl, cd, file_name;
    radians=false,
    viterna_extrapolation=false, rotation_correction=false, aligned = false,
    save_figure=true,
    extension=".pdf", tag="", clear_figure=true, close_figure=true
)

    # set up units
    aoaunits = L" [^\circ]"
    conversionfactor = radians ? 180.0 / pi : 1

    # set up figure
    fig = plt.figure(file_name)
    if !clear_figure && length(fig.get_axes()) == 2
        axes = fig.get_axes()
    else
        fig.clear()
        fig.suptitle(splitpath(file_name)[end])
        fig.add_subplot(211)
        fig.add_subplot(212)
        axes = fig.get_axes()
        axes[1].set_ylabel(L"c_l")
        axes[2].set_ylabel(L"c_d")
        axes[2].set_xlabel(L"\alpha" * aoaunits)
    end

    # set labels
    labeltag = ""
    viterna_extrapolation ? labeltag *= "Ext." : labeltag *= "Not ext."
    rotation_correction ? labeltag *= " w/ rot. corr." : labeltag *= " w/o rot. corr."
    if aligned; labeltag *= ", aoa ali."; end
    labeltag *= tag

    # plot data
    axes[1].plot(α .* conversionfactor, cl, label = labeltag)
    axes[2].plot(α .* conversionfactor, cd, label = labeltag)

    # show figure
    axes[1].legend(loc="upper left", bbox_to_anchor=(1.03,0.9), prop=Dict("size" => 9))
    fig.subplots_adjust(left=0.1, right=0.7, top=0.9, bottom=0.1)
    # fig.show()

    # save plot
    if save_figure
        save_name = splitext(file_name)[1] * extension
        save_path = joinpath(splitpath(file_name)[1:end-1]...)
        if !isdir(save_path); mkpath(save_path); end
        fig.savefig(joinpath(save_path, save_name), bbox_inches="tight")
    end

    if close_figure; plt.close(file_name); end

    return nothing
end


"Overload `plotairfoil` to accept a CCBlade.AlphaAF object."
function plot_airfoil(airfoilobject::CCBlade.AlphaAF, file_name; kwargs...)
    plot_airfoil(airfoilobject.alpha, airfoilobject.cl, airfoilobject.cd, file_name; kwargs...)
end


"""
    plotrotor(file_name, rotorname, rs, chords, twists;
        cratio=1.0, savefigure=true, savepath=joinpath(topdirectory, "data", "plots", TODAY),
        extension=".pdf", clearfigure=true, closefigure=true)

Plots rotor geometry.

# Arguments:

* `file_name::String`: used to save the file
* `rotorname::String`: label for the current plot series
* `rs::Vector{Float64}`: radial stations of the rotor
* `chords::Vector{Float64}`: chord lengths at each radial station
* `twists::Vector{Float64}`: twists at each radial station

# Keyword Arguments:

* `cratio`: value between 0 and 1 used to set the plot color
* `savefigure`: save the figure
* `savepath`: directory to save the figure
* `extension`: file extension to the saved file
* `clearfigure`: clear the figure before plotting
* `closefigure`: close the figure upon completion

"""
function plotrotor(file_name, rotorname, rs, chords, twists;
            cratio=1.0, savefigure=true, savepath=joinpath(topdirectory, "data", "plots", TODAY),
            extension=".pdf", clearfigure=true, closefigure=true)

    # set up units
    aoaunits = L" [^\circ]"
    conversionfactor = radians ? 180.0 / pi : 1

    # set up figure
    fig = plt.figure(file_name)
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
        savename = splitext(file_name)[1] * extension
        fig.savefig(joinpath(savepath, savename), bbox_inches="tight")
    end

    if closefigure; plt.close(file_name); end

    return nothing
end


"""
    rotorinflow(freestream, orientation)

# Arguments:

* `freestream::Freestream`: freestream object
* `orientation::Vector{Float64}`: unit vector aligned with the rotor axis of rotation in the direction of positive thrust

# Returns:

* `vinflow::Float64`: the magnitude of the freestream in the negative thrust direction.
"""
function rotorinflow(freestream, orientation)#, interstream)

    Vinf = freestream2vector(freestream)
    vinflow = -LA.dot(Vinf, orientation)

    return vinflow
end


"""
    rotor2operatingpoints(vinflow, r, omega, environment)

# Arguments:

* `vinflow`
* `r::Vector{Float64}`: vector of a rotor's radial sections
* `omega::Float64`: rotational velocity in rad/s
* `environment::Environment`: `Environment` struct

# Returns:

* `operatingpoints::Vector{CCBlade.OperatingPoint}`: vector of operating points for a single rotor

"""
function rotor2operatingpoints(vinflow, r, omega, environment)

    operatingpoints = CC.simple_op.(vinflow, omega, r, environment.ρ; pitch = 0.0, mu = environment.μ, asound = environment.a, precone = 0.0)

    return operatingpoints
end


"""
    solverotorsnondimensional(rotor_system, omegas, freestream, environment)

# Arguments:

* `rotor_system`
* `omegas`
* `freestream`
* `environment`

# Returns:

* `Js`
* `CTs`
* `CQs`
* `ηs`

"""
function solverotorsnondimensional(rotor_system, omegas, freestream, environment)#, interstream::Interstream)

    # extract rotor info
    orientations = rotor_system.orientations

    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)

    # pre-allocate solution
    Js, Ts, Qs, Ps, us, vs = solverotors(rotor_system, omegas, freestream, environment)

    CTs = similar(Qs)
    CQs = similar(Qs)
    ηs = similar(Qs)

    # iterate over rotors
    for (i, rotorindex) in enumerate(rotor_system.index)

        # isolate rotor
        rotor = rotor_system.rotors[rotorindex]

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
function solverotorsnondimensional!(Js, Ts, Qs, Ps, CTs, CQs, ηs, rotor_system, omegas, freestream, environment)#, interstream::Interstream)

    # extract rotor info
    orientations = rotor_system.orientations

    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)

    # pre-allocate solution
    solve_rotors!(Js, Ts, Qs, Ps, rotor_system, omegas, freestream, environment)

    # iterate over rotors
    for (i, rotorindex) in enumerate(rotor_system.index)
        # isolate rotor
        rotor = rotor_system.rotors[rotorindex]
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
function solverotorsnondimensional!(Js, Ts, Qs, Ps, CTs, CQs, ηs, us, vs, rotor_system, omegas, freestream, environment)#, interstream::Interstream)

    # extract rotor info
    orientations = rotor_system.orientations

    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)

    # pre-allocate solution
    solve_rotors!(Js, Ts, Qs, Ps, us, vs, rotor_system, omegas, freestream, environment)

    # iterate over rotors
    for (i, rotorindex) in enumerate(rotor_system.index)
        # isolate rotor
        rotor = rotor_system.rotors[rotorindex]
        η, CT, CQ = CC.nondim(Ts[i], Qs[i], vinflows[i], omegas[i], environment.ρ, rotor, "propeller")
        CTs[i] = CT
        CQs[i] = CQ
        ηs[i] = η
    end

    return nothing
end


"""
    solverotors(rotor_system, omegas, freestream, environment)

Solves the rotor system at the specified RPM, freestream, and environment.

# Arguments:

* `rotor_system::CCBladeSystem`: a rotor system
* `omegas::Vector{Float64}: vector of rotational velocities in rad/s of each rotor
* `freestream::Freestream`: Freestream object
* `environment::Environment`: Environment object

# Returns:

* `Js::Vector{Float64}`: vector of advance ratios of each rotor
* `Ts::Vector{Float64}`: vector of thrust values of each rotor
* `Qs::Vector{Float64}`: vector of torque values of each rotor
* `Ps::Vector{Float64}`: vector of power supplied at each rotor
* `us::Vector{Vector{Float64}}`: [i][j] refers to the axial induced velocity at the rotor disk at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}`: [i][j] refers to the swirl induced velocity at the rotor disk at the jth radial station of the ith rotor

"""
function solverotors(rotor_system, omegas, freestream, environment)#, interstream::Interstream)

    # pre-allocate solution
    Js = zeros(length(rotor_system.index))
    Ts = similar(Js)
    Qs = similar(Js)
    Ps = similar(Js)
    us = [zeros(typeof(freestream.vinf), length(radii)) for radii in rotor_system.rlists[rotor_system.index]]
    vs = deepcopy(us)

    solve_rotors!(Js, Ts, Qs, Ps, us, vs, rotor_system, omegas, freestream, environment)

    return Js, Ts, Qs, Ps, us, vs
end


"""
    solve_rotors!(Js, Ts, Qs, us, vs, rotor_system, omegas, freestream, environment)

In-place instance of `solverotors`.

# Arguments:

* `Js::Vector{Float64}` : vector of advance ratios of each rotor
* `Ts::Vector{Float64}` : vector of thrust values of each rotor
* `Qs::Vector{Float64}` : vector of torque values of each rotor
* `us::Vector{Vector{Float64}}` : [i][j] refers to the axial induced velocity at the rotor disk at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}` : [i][j] refers to the swirl induced velocity at the rotor disk at the jth radial station of the ith rotor
* `rotor_system::CCBladeSystem` : a rotor system
* `omegas::Vector{Float64} : vector of rotational velocities in rad/s of each rotor
* `freestream::Freestream` : Freestream object
* `environment::Environment` : Environment object

# Modifies:

* `Js::Vector{Float64}`
* `Ts::Vector{Float64}`
* `Qs::Vector{Float64}`
* `us::Vector{Vector{Float64}}`
* `vs::Vector{Vector{Float64}}`

"""
function solve_rotors!(Js, Ts, Qs, Ps, us, vs, rotor_system, omegas, freestream, environment)#, interstream::Interstream)

    # benchmarks = Vector{Float64}(undef,5)

    # benchmarks[1] = @elapsed begin
    # extract rotor info
    rlists = rotor_system.rlists[rotor_system.index]
    orientations = rotor_system.orientations
    # end

    # benchmarks[2] = @elapsed begin
    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)
    operatingpoints_list = rotor2operatingpoints.(vinflows, rlists, omegas, Ref(environment))
    # end

    # iterate over rotors
    for (i, rotorindex) in enumerate(rotor_system.index)
        # benchmarks[3] = @elapsed begin
        # isolate rotor
        rotor = rotor_system.rotors[rotorindex]
        sections = rotor_system.sectionlists[rotorindex]

        # isolate operating conditions
        operatingpoints = operatingpoints_list[i]
        # end

        # benchmarks[4] = @elapsed begin
        # solve CCBlade
        outputs = CC.solve.(Ref(rotor), sections, operatingpoints)
        # end

        # benchmarks[5] = @elapsed begin
        T, Q = CC.thrusttorque(rotor, sections, outputs)
        Ts[i] = T
        Qs[i] = Q
        Ps[i] = Q * omegas[i]
        us[i][:] .= outputs.u
        vs[i][:] .= outputs.v
        # end
    end

    # calculate Js
    Js .= vinflows ./ omegas * 2 * pi ./ [2 * rotor.Rtip for rotor in rotor_system.rotors[rotor_system.index]]

    # println("CHRONOS: $benchmarks")
    return nothing
end


"""
    solve_rotors!(Js, Ts, Qs, rotor_system, omegas, freestream, environment)

In-place instance of `solverotors`.

# Arguments:

* `Js::Vector{Float64}` : vector of advance ratios of each rotor
* `Ts::Vector{Float64}` : vector of thrust values of each rotor
* `Qs::Vector{Float64}` : vector of torque values of each rotor
* `us::Vector{Vector{Float64}}` : [i][j] refers to the axial induced velocity at the rotor disk at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}` : [i][j] refers to the swirl induced velocity at the rotor disk at the jth radial station of the ith rotor
* `rotor_system::CCBladeSystem` : a rotor system
* `omegas::Vector{Float64} : vector of rotational velocities in rad/s of each rotor
* `freestream::Freestream` : Freestream object
* `environment::Environment` : Environment object

# Modifies:

* `Js::Vector{Float64}`
* `Ts::Vector{Float64}`
* `Qs::Vector{Float64}`
* `us::Vector{Vector{Float64}}`
* `vs::Vector{Vector{Float64}}`

"""
function solve_rotors!(Js, Ts, Qs, Ps, rotor_system, omegas, freestream, environment)#, interstream::Interstream)

    # extract rotor info
    rlists = rotor_system.rlists[rotor_system.index]
    orientations = rotor_system.orientations

    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)
    operatingpoints_list = rotor2operatingpoints.(vinflows, rlists, omegas, Ref(environment))

    # iterate over rotors
    for (i, rotorindex) in enumerate(rotor_system.index)
        # isolate rotor
        rotor = rotor_system.rotors[rotorindex]
        sections = rotor_system.sectionlists[rotorindex]

        # isolate operating conditions
        operatingpoints = operatingpoints_list[i]

        # solve CCBlade
        outputs = CC.solve.(Ref(rotor), sections, operatingpoints)
        T, Q = CC.thrusttorque(rotor, sections, outputs)
        Ts[i] = T
        Qs[i] = Q
        Ps[i] = Q * omegas[i]
    end

    # calculate Js
    Js .= vinflows ./ omegas * 2 * pi ./ [2 * rotor.Rtip for rotor in rotor_system.rotors[rotor_system.index]]

    return Js, Ts, Qs, Ps
end


solverotor(rotor, sections, operatingpoints) = CC.solve.(Ref(rotor), sections, operatingpoints)


"""
    solverotors_outputs(rotor_system, omegas, freestream, environment)

# Arguments
* `rotor_system`
* `omegas`
* `freestream`
* `environment`

# Returns

* `Js`
* `outputs`

"""
function solverotors_outputs(rotor_system, omegas, freestream, environment)

    # extract rotor info
    rlists = rotor_system.rlists[rotor_system.index]
    orientations = rotor_system.orientations

    # operating conditions
    vinflows = rotorinflow.(Ref(freestream), orientations)
    operatingpoints_list = rotor2operatingpoints.(vinflows, rlists, omegas, Ref(environment))

    # solve rotors
    outputs_list = solverotor.(rotor_system.rotors[rotor_system.index], rotor_system.sectionlists[rotor_system.index], operatingpoints_list)

    # calculate Js
    Js = vinflows ./ omegas * 2 * pi ./ [2 * rotor.Rtip for rotor in rotor_system.rotors[rotor_system.index]]

    return Js, outputs
end


"""
    induced2wakefunction(rotor_system, us, vs;
        wake_shape_functions = Function[(Rtip, x) -> Rtip for i in 1:length(rotor_system.index)],
        axial_interpolations = Function[(rs, us, r, Rtip) -> FM.linear(rs, us, r) for i in 1:length(rotor_system.index)],
        swirl_interpolations = Function[(rs, vs, r, Rtip) -> FM.linear(rs, vs, r) for i in 1:length(rotor_system.index)],
        axial_multipliers = Function[(distance2plane, Rtip) -> 2 for i in 1:length(aircraft.rotor_system.index)],
        swirl_multipliers = Function[(distance2plane, Rtip) -> 1 for i in 1:length(aircraft.rotor_system.index)]
    )

Interpolates rotor-induced velocities into a wake function of X.

# Arguments:

* `rotor_system::CCBladeSystem`: system of rotors
* `us::Vector{Vector{Float64}}`: [i][j]th element is the axial induced velocity at the jth radial station of the ith rotor
* `vs::Vector{Vector{Float64}}`: [i][j]th element is the swirl induced velocity at the jth radial station of the ith rotor

# Keyword Arguments:

* `wakeshapefunction = (Rtip, x) -> Rtip,` : function accepts a rotor radius and axial coordinate and returns the local wake radius
* `axial_interpolations = (rs, us, r) -> FM.linear(rs, us, r)` : function accepts radial stations, axial velocities, and the radial coordinate and returns the unmodified axial induced velocity
* `swirl_interpolations = (rs, vs, r) -> FM.linear(rs, vs, r)` : function accepts radial stations, swirl velocities, and the radial coordinate and returns the unmodified swirl induced velocity
* `axialmultiplier = (u, distance2plane, Rtip) -> u * 2` : function multiplied by the axial interpolation function
* `swirlmultiplier = (v, distance2plane, Rtip) -> v` : function multiplied by the swirlinterpolation function

"""
function induced2wakefunction(rotor_system, us, vs;
    wake_shape_functions = Function[(Rtip, x) -> Rtip for i in 1:length(rotor_system.index)],
    axial_interpolations = Function[(rs, us, r, Rtip) -> FM.linear(rs, us, r) for i in 1:length(rotor_system.index)],
    swirl_interpolations = Function[(rs, vs, r, Rtip) -> FM.linear(rs, vs, r) for i in 1:length(rotor_system.index)],
    axial_multipliers = Function[(distance2plane, Rtip) -> 2 for i in 1:length(aircraft.rotor_system.index)],
    swirl_multipliers = Function[(distance2plane, Rtip) -> 1 for i in 1:length(aircraft.rotor_system.index)]
)
    function wakefunction(X)
        Vwake = zeros(3)
        for (irotor, rotorindex) in enumerate(rotor_system.index)

            # translate origin to rotor center (R is the vector from the rotor center to X)
            R = X - rotor_system.positions[irotor]
            r = LA.norm(R)

            # calculate distance from rotor plane (positive is in the negative thrust direction)
            distance2plane = -LA.dot(R, rotor_system.orientations[irotor])

            # angle between normal and R (\in [0,\pi])
            θ = acos(distance2plane/r)

            # calculate distance to centerline
            distance2centerline = r * sin(θ) # always positive

            # check if X is within the wake
            Rtip = rotor_system.rotors[rotorindex].Rtip
            Rhub = rotor_system.rotors[rotorindex].Rhub
            if distance2centerline < wake_shape_functions[irotor](Rtip, distance2plane) && distance2centerline > Rhub #! Quick fix, not as comprehensive. Do we need another function? Or perhaps the wakeshapefunction outputs a lower and upper bound on the radius
                # interpolate wake velocity
                u = axial_interpolations[irotor](rotor_system.rlists[rotorindex], us[irotor], distance2centerline, Rtip) * axial_multipliers[irotor](distance2plane, Rtip)
                v = swirl_interpolations[irotor](rotor_system.rlists[rotorindex], vs[irotor], distance2centerline, Rtip) * swirl_multipliers[irotor](distance2plane, Rtip)
                # get axial unit vector
                axialhat = -rotor_system.orientations[irotor]

                # get tangential unit vector
                tangentialhat = LA.cross(rotor_system.orientations[irotor], R)
                tangentialhat ./= LA.norm(tangentialhat)

                # project R onto the plane parallel to normal
                Vinduced = u * axialhat + v * tangentialhat * (rotor_system.spin_directions[irotor] ? 1 : -1)
                Vwake .+= Vinduced
            end
        end

        return Vwake
    end
end


"""
    `get_omega(;Vinf=nothing, D=nothing, J=nothing)`

Returns rotational velocity in [rad/s] given velocity and advance ratio.
"""
function get_omega(; Vinf=nothing, D=nothing, J=nothing)

    @assert Vinf != nothing "User must supply a velocity with the `Vinf` keyword argument."
    @assert D != nothing "User must supply a diameter with the `D` keyword argument."
    @assert J != nothing "User must supply an advance ratio with the `J` keyword argument."

    return 2pi * Vinf / (D * J)
end


"""
    `get_J(;Vinf=nothing, D=nothing, omega=nothing, n=nothing)`

Returns advance ratio given velocity and rotational velocity. Must supply either `omega` or `n`.
"""
function get_J(;Vinf=nothing, D=nothing, omega=nothing, n=nothing)

    @assert Vinf != nothing "User must supply a velocity with the `Vinf` keyword argument."
    @assert D != nothing "User must supply a diameter with the `D` keyword argument."

    if n == nothing
        @assert omega != nothing "User must supply either `omega` or `n` as a keyword argument."
        n = omega / (2pi)
    end

    return Vinf / (n * D)
end


"""
    `get_Vinf(;D=nothing, J=nothing, omega=nothing, n=nothing)`

Returns velocity in [m/s] given advance ratio and rotational velocity. Must supply either `omega` or `n`.
"""
function get_Vinf(;D=nothing, J=nothing, omega=nothing, n=nothing)

    @assert D != nothing "User must supply a diameter with the `D` keyword argument."
    @assert J != nothing "User must supply an advance ratio with the `J` keyword argument."

    if n == nothing
        @assert omega != nothing "User must supply either `omega` or `n` as a keyword argument."
        n = omega / (2pi)
    end

    return n * D / J
end


"""
    omega_to_rpm(omega)

Convenience function to translate rotational velocity to rotations per minute.

# Arguments

* `omega`: rotational velocty (rad/sec)

# Returns

* `rpm`: rotations per minute

"""
function omega_to_rpm(omega)
    return omega * 30/pi
end


"""
    rpm_to_omega(rpm)

Convenience function to translate rotations per minute to rotational velocity.

# Arguments

* `rpm`: rotations per minute

# Returns

* `omega`: rotational velocty (rad/sec)

"""
function rpm_to_omega(rpm)
    return rpm * pi/30
end
