#=##############################################################################################
Filename: rotor.jl
Author: Ryan Anderson
Contact: rymanderson@gmail.com
README: this file is meant to provide convenience functions for building CCBlade rotor objects.
        This includes running all necessary airfoil analysis and store files in an accessible
        directory structure.
=###############################################################################################

"""
Build a vector of airfoil functions for each rotor section.

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
* `plotoutput = true` : toggle plotting polars
* `saveplots = true` : toggle saving plots
* `plotextension = ".pdf"` : extension of saved plots
* `verbose = true` : toggle verbose output
* `v_lvl = 0` : set verbosity level

Outputs:

* vector of `CCBlade.AlphaReAF` objects
* if `savefiles == true`, writes files storing airfoil data for quick reference later
* if `saveplots == true`, saves plots to `polardirectory` for reference

"""
function rotor2airfoil(radii::Vector, chords::Vector, cr75,
        vinf_low, vinf_high, rpm_low, rpm_high, N_Re::Vector,
        contourfiles::Vector{String}, airfoilnames::Vector{String};
        skipstart = 1,
        xfoil_alpha = range(-20.0, stop=20.0, length= 161), # every quarter degree
        M = 0, ν = NU, Re_digits = -4,
        runxfoil=true, xfoil_iter = 300, xfoil_npan = 200,
        xfoil_clmaxstop = true, xfoil_clminstop = true,
        viternaextrapolation=true, rotationcorrection=true, rotationcorrection_J = 2.0,
        radians = false, savefiles = true, useoldfiles = true, polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        plotoutput = true, saveplots = true, plotextension = ".pdf",
        verbose = true, v_lvl = 0
    ) where T

    @assert length(radii) == length(chords) "length of radii and chords inconsistent"
    @assert length(N_Re) == length(radii) "length of radii and N_Re inconsistent"
    @assert length(radii) == length(contourfiles) "length of radii and contourfiles inconsistent"
    @assert length(radii) == length(airfoilnames) "length of radii and airfoilnames inconsistent"

    airfoilobjects = Array{CC.AlphaReAF,1}(undef,length(radii))

    # get polars for each radial station
    for (i_radius, radius) in enumerate(radii)
        Res = Re_range(radius, chords[i_radius], vinf_low, vinf_high, rpm_low, rpm_high, N_Re[i_radius];
                ν = ν, Re_digits = Re_digits
            )
        # check if files already exist
        filenames_uncorrected = airfoilfilenames(airfoilnames[i_radius], Res; M=0, viternaextrapolation=false, rotationcorrection=false, aoaset=false)
        filepaths_uncorrected = joinpath.(Ref(polardirectory), filenames_uncorrected)
        i_existingfiles_uncorrected = isfile.(filepaths_uncorrected)
        if runxfoil && (!useoldfiles || !prod(i_existingfiles_uncorrected))
            airfoil2xfoil(Res, contourfiles[i_radius], airfoilnames[i_radius];
                    skipstart = skipstart, # 1 line header in .dat contour file
                    xfoil_alpha = xfoil_alpha, # every quarter degree
                    M = M, ν = ν, Re_digits = Re_digits,
                    xfoil_iter = xfoil_iter, xfoil_npan = xfoil_npan, xfoil_clmaxstop = xfoil_clmaxstop, xfoil_clminstop = xfoil_clminstop,
                    radians = radians, useoldfiles = useoldfiles, polardirectory = polardirectory,
                    plotoutput = plotoutput, saveplots = saveplots, plotextension = plotextension,
                    verbose = verbose, v_lvl = v_lvl
                )
        end
        airfoilobjects[i_radius] = correct_align_polars(Res, airfoilnames[i_radius], cr75;
                M = M,
                viternaextrapolation = viternaextrapolation, rotationcorrection = rotationcorrection, rotationcorrection_J = rotationcorrection_J,
                radians = radians, savefiles = savefiles, polardirectory = polardirectory,
                plotoutput = plotoutput, saveplots = saveplots, plotextension = ".pdf",
            )
    end

    return airfoilobjects
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
* `plotoutput = true` : toggle plotting polars
* `saveplots = true` : toggle saving plots
* `plotextension = ".pdf"` : extension of saved plots
* `verbose = true` : toggle verbose output
* `v_lvl = 0` : set verbosity level

Outputs:

* writes polar files in `polardirectory`
* saves plots to `polardirectory`
* returns `nothing`

"""
function airfoil2xfoil(Res, contourfile::String, airfoilname::String;
        skipstart = 1, # 1 line header in .dat contour file
        xfoil_alpha = range(-20.0, stop=20.0, length= 161), # every quarter degree
        M = 0, ν = NU, Re_digits = -4,
        xfoil_iter = 300, xfoil_npan = 200, xfoil_clmaxstop = true, xfoil_clminstop = true,
        radians = false, useoldfiles = true, polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        plotoutput = true, saveplots = true, plotextension = ".pdf",
        verbose = true, v_lvl = 0
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
                    clminstop=clminstop
                    )
            # create object and save files
            angleconversion = radians ? pi / 180.0 : 1
            thisobject = CC.AlphaAF(xfoil_alpha[conv] .* angleconversion, cl[conv], cd[conv], airfoilname, Re * 1.0, M * 1.0)
            CC.write_af(filepaths_uncorrected[i_Re], thisobject)
            if plotoutput
                plot_airfoil(thisobject, filenames_uncorrected[i_Re], airfoilname, Re, M;
                    viternaextrapolation = false, rotationcorrection = false,
                    savefigure = saveplots, savepath = polardirectory,
                    extension = plotextension)
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
* `rotationcorrection_J = 2.0` : advance ratio at which to perform the rotational correction
* `radians = false` : airfoil files should be in radians
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
function correct_align_polars(Res, airfoilname::String, cr75;
        M = 0,
        viternaextrapolation=true, rotationcorrection=true, rotationcorrection_J = 2.0,
        radians = false, savefiles = true, polardirectory = joinpath(topdirectory, "data", "airfoil", "polars", TODAY),
        plotoutput = true, saveplots = true, plotextension = ".pdf",
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
                plot_airfoil(α, cl, cd, filenames[i_Re], airfoilname, Re, M;
                    viternaextrapolation = true, rotationcorrection = false,
                    savefigure = saveplots, savepath = polardirectory,
                    extension = plotextension, tag = "", clearfigure = true
                )
            end
        end
        if rotationcorrection
            for (i_α, this_α) in enumerate(α)
                cl[i_α], cd[i_α] = CC.rotation_correction(CC.DuSeligEggers(), cl[i_α], cd[i_α], cr75, 0.75, pi/rotationcorrection_J, this_α)
            end
            if plotoutput
                plot_airfoil(α, cl, cd, filenames[i_Re], airfoilname, Re, M;
                    viternaextrapolation = viternaextrapolation, rotationcorrection = true,
                    savefigure = saveplots, savepath = polardirectory,
                    extension = plotextension, tag = "", clearfigure = false
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

function airfoilfilenames(airfoilname, Res;
    M=0, viternaextrapolation=false, rotationcorrection=false,
    aoaset=false, extension = ".txt"
    )
    tags = ""
    if viternaextrapolation; tags *= "_ext"; end
    if rotationcorrection; tags *= "_rot"; end
    if aoaset; tags *= "_aoaset"; end
    [airfoilname * "_Re$(Re)" * tags * extension for Re in Res]
end

function Re_range(radius, chord, vinf_low, vinf_high, rpm_low, rpm_high, N_Re::Int; ν = Nu, Re_digits=-4)
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
* savepath = joinpath(topdirectory, "data", "airfoil", "plots", TODAY) : directory to save the figure
* extension = ".pdf" : figure extension
* tag = "" : string to append to the series legend
* clearfigure = true : clear the figure before plotting (set false if you want to compare series)

"""
function plot_airfoil(α::Vector, cl::Vector, cd::Vector, filename::String, airfoilname::String, Re, M;
        radians = false,
        viternaextrapolation = false, rotationcorrection = false,
        savefigure = false, savepath = joinpath(topdirectory, "data", "airfoil", "plots", TODAY),
        extension = ".pdf", tag = "", clearfigure = true
    )
    # set up units
    aoaunits = radians ? "" : L" [^\circ]"
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
    axes[1].plot(α, cl, label = labeltag)
    axes[2].plot(α, cd, label = labeltag)
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
    return nothing
end

"Overload `plot_airfoil` to accept a CCBlade.AlphaAF object."
function plot_airfoil(airfoilobject::CCBlade.AlphaAF, filename::String, airfoilname::String, Re, M;
    viternaextrapolation = false, rotationcorrection = false,
    savefigure = false, savepath = joinpath(topdirectory, "data", "airfoil", "plots", TODAY),
    extension = ".pdf", tag = "", clearfigure = true
    )
    plot_airfoil(airfoilobject.alpha, airfoilobject.cl, airfoilobject.cd, filename, airfoilname, Re, M;
        viternaextrapolation = viternaextrapolation, rotationcorrection = rotationcorrection,
        savefigure = savefigure, savepath = savepath,
        extension = extension, tag = tag, clearfigure = clearfigure
        )
end
