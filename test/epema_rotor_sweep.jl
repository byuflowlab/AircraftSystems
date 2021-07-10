# Epema propeller
include("EpemaData3.jl")

nblades = [6, 6, 6]
rtip = ones(3) * 0.406 / 2
rs_desired = [0.207, 0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0]
radii = fill(rs_desired .* rtip[1], 3)
rhub = fill(radii[1][1], 3)
nJs = 27
Js = ones(3,nJs) .* range(0.01, stop=2.0, length=nJs)'
# Vinf = 19 # m/s
# ns = Vinf ./ Js / 2 / rtip[1]
# omegas = ns .* 2 * pi / 60
omegas = ones(3,nJs) .* 5000 * 2 * pi / 60 # rad/s

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
contourdirectory = joinpath(AS.topdirectory, "test", "data", "airfoil", "contours")
EpemaData3.scans_to_interpolated_contours(contourdirectory, plotdirectory, rs_desired)

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

polardirectory = joinpath(AS.topdirectory, "test", "data", "airfoil", "polars")
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
    rotornames = [L"\theta_{r/R=0.7} = 25^\circ", L"\theta_{r/R=0.7} = 30^\circ", L"\theta_{r/R=0.7} = 35^\circ"],
    plotdirectory = joinpath(AS.topdirectory, "data","plots",TODAY),
    plotbasename = plotbasename,
    plotextension = ".pdf"
    )
objective = AS.runsimulation!(simulationdata...)

epema_data = Vector{Array{Float64,2}}(undef,3)
epema_data[1] = [
    0.39907904834996144 0.4526289577244005;
    0.4973138910207212 0.5378398458549013;
    0.600153491941673 0.6145449943665193;
    0.6999232540291632 0.6710341111346976;
    0.798158096699923 0.724330105648177;
    0.8979278587874134 0.7520958181610362;
    0.9976976208749039 0.751138126418576;
    1.0990023023791249 0.6437990888457081;
    1.2033768227168071 0.27476118939925875;
]
epema_data[2] = [
    0.5981966719565649 0.5397182236147184;
    0.696552040863189 0.6025364977471156;
    0.7979670461445826 0.6600372504094993;
    0.8993738869526614 0.7132826795801419;
    1.0007643988141104 0.7580174617673022;
    1.097527695424323 0.79104765501018;
    1.1988753438008684 0.8134419888656995;
]
epema_data[3] = [
    0.6979053023151386 0.5078563665032071;
    0.7993468421348057 0.5791869205137494;
    0.8976899643314572 0.6356222094085349;
    1.0006337672410712 0.6899322858994442;
    1.0974235983895575 0.7367922804904807;
    1.1987855345944043 0.7666334304565471;
    1.2986003031060718 0.7900907787376703;
    1.3999622393109181 0.8199319287037368;
    1.499756596639299 0.8327509682555072;
    1.601034846992667 0.8189750524312269;
    1.6990922126232961 0.7264740191150731;
    1.8012256915564038 0.35844323905067577
]

epema_ct = Vector{Array{Float64,2}}(undef,3)
epema_ct[1] = [
    0.3985663082437276 0.30549662487946005;
    0.49749103942652323 0.28312439729990363;
    0.5992831541218637 0.2595949855351978;
    0.7010752688172042 0.22912246865959504;
    0.7985663082437275 0.19749276759884285;
    0.9003584229390678 0.15969141755062688;
    0.9992831541218636 0.11417550626808104;
    1.101075268817204 0.06325940212150438;
    1.2014336917562722 0.01311475409836066;
]
epema_ct[2] = [
    0.5992831541218637 0.3270973963355835;
    0.6982078853046594 0.30703953712632603;
    0.7999999999999998 0.28466730954676955;
    0.9003584229390678 0.2588235294117648;
    0.9992831541218636 0.22757955641272906;
    1.0982078853046593 0.19286403085824497;
    1.1985663082437275 0.1581485053037609;
]
epema_ct[3] = [
    0.6967741935483871 0.3579556412729027;
    0.7985663082437275 0.3525554484088718;
    0.897491039426523 0.33905496624879466;
    0.9978494623655914 0.3186113789778207;
    1.0967741935483868 0.2950819672131148;
    1.1985663082437275 0.2619093539054967;
    1.297491039426523 0.22256509161041468;
    1.3992831541218635 0.18707810993249763;
    1.4982078853046594 0.15043394406943106;
    1.5985663082437271 0.11031822565091615;
    1.697491039426523 0.06325940212150438;
    1.7992831541218635 0.015429122468659573;
]

# Epema xrotor data
ct_xrotor = [
    [
        0.3985663082437276 0.32015429122468664;
        0.49749103942652323 0.2989392478302797;
        0.5992831541218637 0.2626808100289297;
        0.6996415770609317 0.22873674059787855;
        0.7985663082437275 0.19749276759884285;
        0.9003584229390678 0.14773384763741565;
        0.9978494623655914 0.10183220829315337;
        1.0996415770609316 0.054387656702025056;
    ],
    [
        0.5992831541218637 0.3502410800385729;
        0.6967741935483871 0.31205400192864036;
        0.7985663082437275 0.2808100289296047;
        0.9003584229390678 0.2464802314368371;
        0.9978494623655914 0.21947926711668278;
        1.0967741935483868 0.18090646094503376;
        1.1985663082437275 0.13654773384763746
    ],
    [
        0.6967741935483871 0.3988428158148506;
        0.7985663082437275 0.37801350048216015;
        0.8974910394265232 0.3529411764705883;
        0.9978494623655914 0.3232401157184186;
        1.0967741935483868 0.2950819672131148;
        1.1985663082437275 0.2595949855351978;
        1.3992831541218638 0.18399228543876572;
        1.496774193548387 0.1307618129218901;
        1.5985663082437271 0.08563162970106075;
        1.697491039426523 0.035486981677917084;
    ]
]

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
for (i,data) in enumerate(epema_data)
    cratio = i / length(epema_data)
    axs[3].scatter(data[:,1], data[:,2], marker = "o", s = 100, facecolors="none", edgecolors = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label="Experiment")
end
axs[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))

# save figure
savename = plotbasename * "_rotorsweep_extratwist_$(round(twist_extra * 180/pi; digits = 0))" * repeattag * ".pdf"
fig.savefig(joinpath(plotdirectory, savename), bbox_inches="tight")
