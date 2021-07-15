# simulation controls
alphas = [0.0, 4.0, 10.0] .* pi/180
ploti = 1:length(alphas)
# wing definition
wing_b = 640e-3 * 2
wing_TR = 1.0 # hershey bar
wing_c = 240e-3
wing_AR = wing_b / wing_c
wing_θroot = 0.0
wing_θtip = 0.0
# environment
environment = AS.Environment()
ν = environment.ν
# freestream definition
Re_c = 0.8e6
vinf = Re_c * ν / wing_c
vinfs = fill(vinf, length(alphas))
# rotor definition
# omegas = ones(1,length(alphas)) .* 1000
# nblades = [4]
# radii = [[0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-3/2]
# rhub = [radii[1][1]]
# rtip = [radii[1][end]]
# chords = [[9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] * 1e-3]
# twists = [[35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0]] # * pi/180 provide in degrees to match airfoil files

# airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "naca4412.dat")
# airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
# airfoilname = "naca4412"
# airfoilnames = [fill(airfoilname, length(radii[1]))]
# index = [1]
# rotor_X = [[-201.8e-3, 300e-3, 0.0]]
# rotor_orientation = [[-1.0, 0.0, 0.0]]
# spindirections = [true]

# polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", "20210524")
# plotstepi = 1:length(alphas)

# Epema propeller w/ 4 blades to match
nblades = [4]
rtip = [236e-3/2]
radii = fill([0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* rtip[1], length(nblades))
rhub = fill(radii[1][1], length(nblades))
J = 0.85
D = 2 * rtip[1]
n = vinf / J / D # rev/s
omega = n .* 2 * pi # rad/s
omegas = ones(length(nblades),length(alphas)) .* omega # rad/s
chords_raw = [
    0.20689655 0.15571575695159628
    0.23119777158774377 0.15571575695159628;
    0.24373259052924792 0.15489186405767247;
    0.254874651810585 0.15427394438722963;
    0.2646239554317549 0.1536560247167868;
    0.2771587743732591 0.152832131822863;
    0.28690807799442897 0.15242018537590113;
    0.29944289693593307 0.1515962924819773;
    0.31058495821727017 0.15097837281153448;
    0.32033426183844016 0.1505664263645726;
    0.33008356545961004 0.14994850669412974;
    0.34122562674094703 0.1493305870236869;
    0.35027855153203336 0.14871266735324407;
    0.3621169916434541 0.14809474768280123;
    0.3746518105849582 0.14747682801235837;
    0.3857938718662953 0.14685890834191553;
    0.39484679665738176 0.14644696189495365;
    0.4032033426183844 0.14582904222451082;
    0.41364902506963797 0.14562306900102984;
    0.42479108635097473 0.14582904222451082;
    0.43454038997214484 0.14603501544799175;
    0.44846796657381605 0.1462409886714727;
    0.4623955431754875 0.14624098867147267;
    0.4798050139275766 0.14644696189495363;
    0.4972144846796658 0.14665293511843458;
    0.5097493036211699 0.14665293511843458;
    0.5250696378830083 0.1474768280123584;
    0.5181058495821728 0.1470648815653965;
    0.5376044568245125 0.14809474768280123;
    0.5536211699164346 0.14891864057672502;
    0.5682451253481894 0.14953656024716785;
    0.5793871866295265 0.15015447991761072;
    0.5919220055710307 0.1505664263645726;
    0.6016713091922006 0.15097837281153448;
    0.6142061281337048 0.1515962924819773;
    0.6267409470752089 0.1518022657054583;
    0.6406685236768802 0.15200823892893922;
    0.6559888579387188 0.15221421215242015;
    0.6740947075208914 0.15262615859938206;
    0.6866295264623956 0.152832131822863;
    0.6991643454038998 0.1532440782698249;
    0.7103064066852367 0.1532440782698249;
    0.7207520891364904 0.15345005149330587;
    0.7381615598885793 0.15303810504634396;
    0.756267409470752 0.15242018537590113;
    0.7743732590529246 0.15200823892893922;
    0.7862116991643455 0.1518022657054583;
    0.7889972144846799 0.1518022657054583;
    0.7924791086350975 0.1511843460350154;
    0.7966573816155988 0.15036045314109164;
    0.8077994428969361 0.1480947476828012;
    0.8217270194986074 0.1454170957775489;
    0.83983286908078 0.1419155509783728;
    0.8565459610027856 0.1388259526261586;
    0.8586350974930362 0.1384140061791967;
    0.8607242339832869 0.13779608650875386;
    0.8662952646239553 0.13553038105046342;
    0.8746518105849582 0.13223480947476826;
    0.8837047353760447 0.12852729145211123;
    0.8920612813370473 0.12523171987641607;
    0.8990250696378829 0.1223480947476828;
    0.9059888579387186 0.11946446961894952;
    0.9129526462395544 0.11658084449021626;
    0.91991643454039 0.113697219361483;
    0.9317548746518106 0.10875386199794025;
    0.9366295264623955 0.10628218331616889;
    0.9456824512534819 0.10175077239958805;
    0.953342618384401 0.09763130792996907;
    0.9610027855153204 0.09351184346035016;
    0.9658774373259055 0.09083419155509764;
    0.9721448467966572 0.08692070030895985;
    0.9770194986072421 0.08362512873326469;
    0.9811977715877437 0.08074150360453139;
    0.9860724233983288 0.07765190525231722;
    0.9902506963788301 0.07456230690010296;
    0.9937325905292481 0.07209062821833162;
    0.998607242339833 0.06920700308959835;
    1.0 0.06817713697219355
] .* rtip[1]
chords = fill(AS.FM.linear(chords_raw[:,1], chords_raw[:,2], radii[1]), length(nblades))

twists_raw = [
    0.20689655          52.651941261622 # tacked on to reach hub radius
    0.2317821454311707  52.651941261622;
    0.254709079221614   51.51808027194226;
    0.272959074351832   50.56838527075396;
    0.2909986948984163  49.651357910884165;
    0.3083545980481913  48.74133075986835;
    0.3257105011979664  47.862415648203495;
    0.3439604963281844  46.91427624898273;
    0.3598963710384323  46.02058291862616;
    0.3772522741882074  45.176095321212834;
    0.3946081773379823  44.29411614506643;
    0.4119640804877574  43.469411404997956;
    0.4311433691656533  42.47811920314493;
    0.4465005746342795  41.74112190590048;
    0.4638564777840545  40.86998480407336;
    0.4808617566277734  40.068849790785926;
    0.4982176597775484  39.20549069879656;
    0.516415061261858   38.32501998516417;
    0.5345072754543507  37.502106544331056;
    0.5527046769386602  36.63096944250393;
    0.5707968911311531  35.77538836035231;
    0.588152794280928   34.935363297876165;
    0.6061924148275124  34.07589321080567;
    0.625283908292265   33.22186773062159;
    0.6433761224847576  32.47751218914967;
    0.661415743031342   31.688821991602623;
    0.6805072364960946  30.89079818225029;
    0.6985994506885872  30.11299719847608;
    0.7166390712351716  29.34375202552339;
    0.7355727837621989  28.71373322866628;
    0.7562420866042037  28.0557135963933;
    0.7753861737148646  27.476251863481515;
    0.7943198862418919  26.900679135488605;
    0.8146210335625379  26.34766263602514;
    0.8337651206731987  25.842869797555686;
    0.8540662679938448  25.297631307929965;
    0.8732103551045057  24.80061647929824;
    0.8917759121101739  24.338602694936363;
    0.9095657128386935  23.90808985041734;
    0.9265052496250269  23.47835480688209;
    0.9425024835888346  23.094315571143582;
    0.9598583867386096  22.69180356204043;
    0.9790550674951789  22.302424660818083;
    1.0000000000000000  21.9829106386545
]
twist = AS.FM.linear(twists_raw[:,1] .* rtip[1], twists_raw[:,2], radii[1]) .* pi/180 # radians
# set collective to 30 deg. at 70\% radius
twists_07_desired = [30.0] .* pi/180
twists_07 = AS.FM.linear(radii[1], twist, 0.7 * rtip[1])
twists = Vector{Vector{typeof(twist[1])}}(undef, length(nblades))
twist_extra = -0.3 * pi/180
for (i, twistval) in enumerate(twists_07_desired)
    twists[i] = twist .+ twistval .- twists_07 .+ twist_extra
end

rs_desired = [0.207, 0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0]
locations = string.(Int.(round.(rs_desired .* 1000, digits=0)))
locations[1:end-1] = "0" .* locations[1:end-1]
# contourfilenames = "epema_interpolated_bspline_n30_" .* locations
# try just repeating the r/R = 0.7 airfoil and see how this looks
contourfilenames = fill("epema_interpolated_bspline_n30_" * locations[8],length(radii[1]))
airfoilcontours = fill(joinpath.(AS.topdirectory, "data", "airfoil", "contours", "20210702", contourfilenames .* ".dat"),length(nblades))
airfoilnames = fill(contourfilenames, length(nblades))

polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", "20210703")
plotstepi = 1:length(alphas)

Res = [5e4, 1e5, 5e5, 1e6, 1e7]
Machs = [0.0, 0.1, 0.2, 0.3]

Res_list = fill(fill(Res, length(radii[1])), length(nblades))
Ms_list = fill(fill(Machs, length(radii[1])), length(nblades))

index = [1]
rotor_X = [[-201.8e-3, 300e-3, 0.0]]
rotor_orientation = [[-1.0, 0.0, 0.0]]
spindirections = [true]

# call template
plotbasename = "PROWIM_propson"
wakedevelopementfactors = [0.0, 0.5, 1.0]
swirlrecoveryfactors = [0.0, 0.5, 1.0]

for wakedevelopementfactor in wakedevelopementfactors
    for swirlrecoveryfactor in swirlrecoveryfactors
        local data_PROWIM_vlm_bem = AS.vlm_bem_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, rotor_X, rotor_orientation, spindirections, Res_list, Ms_list;
                wakedevelopementfactor = wakedevelopementfactor, # fully developed by default
                swirlrecoveryfactor = swirlrecoveryfactor, # as described in Veldhuis' paper
                polardirectory = polardirectory,
                surfacenames = ["wing"],
                rotornames = ["rotor"],
                plotbasename = plotbasename,
                plotextension = ".pdf"
            )
        local objective = AS.runsimulation!(data_PROWIM_vlm_bem...)

        # plot validation
        waketag = "_wdf_$(round(wakedevelopementfactor; digits=1))_srf_$(round(swirlrecoveryfactor; digits=1))"
        fig_rotor = plt.figure(plotbasename * "_rotor_sweep")
        ax_rotor = fig_rotor.get_axes()[1]
        ax_rotor.scatter(0.85, 0.168, c="r", label="Velduis")
        ax_rotor.legend(loc="upper left", bbox_to_anchor=(1.01,1))
        fig_rotor.tight_layout()
        fig_rotor.savefig(joinpath(plotdirectory,"PROWIM_props_on_rotor_sweep" * waketag * ".pdf"), bbox_inches="tight")
        # fig_rotor.savefig(joinpath(notebookdirectory,"PROWIM_propson_rotorsweep.pdf"), bbox_inches="tight")

        fig_cf = plt.figure(plotbasename * "_lift_distribution")
        axs_cf = fig_cf.get_axes()
        cl_data_props_on = PROWIMData.props_on["lift_distribution"]
        plot_labels = "Velduis, " .* [LS.L"\alpha = 0^\circ", LS.L"\alpha = 4^\circ", LS.L"\alpha = 10^\circ"]
        local aircraft = data_PROWIM_vlm_bem[1]
        b = aircraft.wingsystem.lifting_line_rs[1][2,end]
        for (i, data) in enumerate(cl_data_props_on)
            cratio = i / length(cl_data_props_on)
            axs_cf[3].scatter(data[:,1] .* b, data[:,2], marker="+", color=(0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=plot_labels[i])
        end
        axs_cf[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))
        fig_cf.tight_layout()
        fig_cf.savefig(joinpath(plotdirectory,"PROWIM_props_on_lift_distribution" * waketag * ".pdf"), bbox_inches="tight")
        # fig_cf.savefig(joinpath(notebookdirectory,"PROWIM_propson_lift_distribution" * waketag * ".pdf"), bbox_inches="tight")

        fig_CF = plt.figure(plotbasename * "_cl_alpha_sweep")
        axs_CF = fig_CF.get_axes()

        # Fetch data.
        CL_data = PROWIMData.props_on["CL_alpha"]
        CD_data_balance = PROWIMData.props_on["CD_balance"]
        CD_data_wakesurvey = PROWIMData.props_on["CD_wake_survey"]

        axs_CF[1].scatter(CL_data[:,1], CL_data[:,2], marker = "+", label="Velduis")
        axs_CF[1].legend(loc="upper left", bbox_to_anchor=(1.01,1))
        axs_CF[2].scatter(CD_data_balance[:,1], CD_data_balance[:,2], marker = "+", label="Velduis, balance")
        axs_CF[2].scatter(CD_data_wakesurvey[:,1], CD_data_wakesurvey[:,2], marker = "+", label="Velduis, wake survey")
        axs_CF[2].legend(loc="upper left", bbox_to_anchor=(1.01,1))
        fig_CF.tight_layout()
        fig_CF.savefig(joinpath(plotdirectory,"PROWIM_props_on_cl_alpha_sweep" * waketag * ".pdf"), bbox_inches="tight")
        # fig_CF.savefig(joinpath(notebookdirectory,"PROWIM_propson_clalphasweep" * waketag * ".pdf"), bbox_inches="tight")
    end
end
