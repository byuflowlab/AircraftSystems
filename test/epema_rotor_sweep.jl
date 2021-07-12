# Epema propeller
nblades = [6, 6, 6]
D = EpemaData3.rotor["diameter"]
rtip = ones(3) * D / 2
radii = fill(rs_desired .* rtip[1], 3)
rhub = fill(radii[1][1], 3)
nJs = 27
Js = ones(3,nJs) .* range(0.01, stop=2.0, length=nJs)'
# Vinf = 19 # m/s
# ns = Vinf ./ Js / 2 / rtip[1]
# omegas = ns .* 2 * pi / 60
omegas = ones(3,nJs) .* 5000 * 2 * pi / 60 # rad/s, arbitrary RPM

chords_raw = EpemaData3.rotor["chords_raw"]
chords = fill(AS.FM.linear(chords_raw[:,1], chords_raw[:,2], radii[1]), 3)

twists_raw = EpemaData3.rotor["twists_raw"]
twist = AS.FM.linear(twists_raw[:,1] .* rtip[1], twists_raw[:,2], radii[1]) .* pi/180 # radians
# set collective at 70\% radius
twists_07_desired = [25.0, 30.0, 35.0] .* pi/180
twists_07 = AS.FM.linear(radii[1], twist, 0.7 * rtip[1])
twists = Vector{Vector{typeof(twist[1])}}(undef, 3)
twist_extra = 3.0 * pi/180
for (i, twistval) in enumerate(twists_07_desired)
    twists[i] = twist .+ twistval .- twists_07 .+ twist_extra
end

# extract scanned contour data, smooth, and interpolate to rs_desired
locations = string.(Int.(round.(rs_desired .* 1000, digits=0)))
locations[1:end-1] = "0" .* locations[1:end-1]

##########################################################
# uncomment the next 2 lines to use all the distinct polars:
# contourfilenames = "epema_interpolated_bspline_n30_" .* locations
# repeattag = ""
# or uncomment these lines to try just repeating the r/R = 0.7 airfoil:
contourfilenames = fill("epema_interpolated_bspline_n30_" * locations[8],length(locations))
repeattag = "_repeat07af"
##########################################################
airfoilcontours = fill(joinpath.(contourdirectory, contourfilenames .* ".dat"),3)
airfoilnames = fill(contourfilenames, 3)

plotstepi = 1:length(Js)

Res = [5e4, 1e5, 5e5, 1e6, 1e7]
Machs = [0.0, 0.1, 0.2, 0.3]

Res_list = fill(fill(Res, length(radii[1])), 3)
Ms_list = fill(fill(Machs, length(radii[1])), 3)

index = [1,2,3]
positions = [[0.0, 0.0, -10.0], [0.0, 0.0, 0.0], [0.0, 0.0, 10.0]]
orientations = fill([-1.0, 0.0, 0.0],3)
spindirections = fill(true,3)

plotbasename = "epema"
simulationdata = AS.rotor_sweep_template(Js, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, positions, orientations, spindirections, Res_list, Ms_list;
    polardirectory = polardirectory,
    closefigure = true,
    useoldfiles = true,
    rotornames = [LS.L"\theta_{r/R=0.7} = 25^\circ", LS.L"\theta_{r/R=0.7} = 30^\circ", LS.L"\theta_{r/R=0.7} = 35^\circ"],
    plotdirectory = joinpath(AS.topdirectory, "data", "plots", AS.TODAY),
    plotbasename = plotbasename,
    plotextension = ".pdf"
    )
objective = AS.runsimulation!(simulationdata...)

epema_eta = EpemaData3.epema_eta

epema_ct = EpemaData3.epema_ct

# Epema xrotor data
ct_xrotor = EpemaData3.ct_xrotor

fig = plt.figure(plotbasename * "_rotorsweep")
axs = fig.get_axes()

# plot CTs
for (i,data) in enumerate(epema_ct)
    cratio = i / length(epema_ct)
    axs[1].scatter(data[:,1], data[:,2], marker = "o", s = 100, facecolors="none", edgecolors = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="Experiment")
    axs[1].scatter(ct_xrotor[i][:,1], marker = "x", ct_xrotor[i][:,2], color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="XROTOR")
end
axs[1].legend(loc="upper left", bbox_to_anchor=(1.01,1))

# plot efficiencies
for (i,data) in enumerate(epema_eta)
    cratio = i / length(epema_eta)
    axs[3].scatter(data[:,1], data[:,2], marker = "o", s = 100, facecolors="none", edgecolors = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="Experiment")
end
axs[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))

# save figure
savename = plotbasename * "_rotorsweep_extratwist_$(round(twist_extra * 180/pi; digits = 0))" * repeattag * ".pdf"
fig.savefig(joinpath(plotdirectory, savename), bbox_inches="tight")
