import AircraftSystems
AS = AircraftSystems
import FileIO
FM = AS.FM
using LaTeXStrings
plt = AS.plt
VL = AS.VL
using Test
using PyPlot; close("all")
using Infiltrator

# # topdirectory = normpath(joinpath(@__DIR__,".."))

# # @testset "rotors" begin

# # # test rotor_sweep_template
# nJs = 17
# Js = ones(1,nJs) .* range(0.01, stop=2.0, length=nJs)'
# # omegas = fill(50.0, length(Js))
# # nblades = 3
# # radii = [0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-3/2
# # rhub = radii[1]
# # rtip = radii[end]
# # chords = [9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] * 1e-3
# # twists = [35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0] # * pi/180 provide in degrees to match airfoil files
# # airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "naca4412.dat")
# # airfoilcontours = fill(airfoilcontour, length(radii))
# # airfoilname = "naca4412"
# # airfoilnames = fill(airfoilname, length(radii))
# # polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", "20210524")


# # # PROWIM propeller
# # omegas = ones(1,length(Js)) .* 5000 * 2 * pi / 60 # rad/s
# # nblades = [4]
# # radii = [[0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-3/2]
# # rhub = [radii[1][1]]
# # rtip = [radii[1][end]]
# # chords = [[9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] .* 1e-3]
# # twists = [[35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0]] * pi/180 # provide in degrees to match airfoil files

# # Epema propeller
# omegas = ones(1,length(Js)) .* 5000 * 2 * pi / 60 # rad/s
# nblades = [6]
# rtip = [0.406 / 2]
# radii = [[0.207, 0.3, 0.4, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95, 1.0] .* rtip[1]]
# rhub = [radii[1][1]]

# chords_raw = [
#     0.20689655 0.15571575695159628
#     0.23119777158774377 0.15571575695159628;
#     0.24373259052924792 0.15489186405767247;
#     0.254874651810585 0.15427394438722963;
#     0.2646239554317549 0.1536560247167868;
#     0.2771587743732591 0.152832131822863;
#     0.28690807799442897 0.15242018537590113;
#     0.29944289693593307 0.1515962924819773;
#     0.31058495821727017 0.15097837281153448;
#     0.32033426183844016 0.1505664263645726;
#     0.33008356545961004 0.14994850669412974;
#     0.34122562674094703 0.1493305870236869;
#     0.35027855153203336 0.14871266735324407;
#     0.3621169916434541 0.14809474768280123;
#     0.3746518105849582 0.14747682801235837;
#     0.3857938718662953 0.14685890834191553;
#     0.39484679665738176 0.14644696189495365;
#     0.4032033426183844 0.14582904222451082;
#     0.41364902506963797 0.14562306900102984;
#     0.42479108635097473 0.14582904222451082;
#     0.43454038997214484 0.14603501544799175;
#     0.44846796657381605 0.1462409886714727;
#     0.4623955431754875 0.14624098867147267;
#     0.4798050139275766 0.14644696189495363;
#     0.4972144846796658 0.14665293511843458;
#     0.5097493036211699 0.14665293511843458;
#     0.5250696378830083 0.1474768280123584;
#     0.5181058495821728 0.1470648815653965;
#     0.5376044568245125 0.14809474768280123;
#     0.5536211699164346 0.14891864057672502;
#     0.5682451253481894 0.14953656024716785;
#     0.5793871866295265 0.15015447991761072;
#     0.5919220055710307 0.1505664263645726;
#     0.6016713091922006 0.15097837281153448;
#     0.6142061281337048 0.1515962924819773;
#     0.6267409470752089 0.1518022657054583;
#     0.6406685236768802 0.15200823892893922;
#     0.6559888579387188 0.15221421215242015;
#     0.6740947075208914 0.15262615859938206;
#     0.6866295264623956 0.152832131822863;
#     0.6991643454038998 0.1532440782698249;
#     0.7103064066852367 0.1532440782698249;
#     0.7207520891364904 0.15345005149330587;
#     0.7381615598885793 0.15303810504634396;
#     0.756267409470752 0.15242018537590113;
#     0.7743732590529246 0.15200823892893922;
#     0.7862116991643455 0.1518022657054583;
#     0.7889972144846799 0.1518022657054583;
#     0.7924791086350975 0.1511843460350154;
#     0.7966573816155988 0.15036045314109164;
#     0.8077994428969361 0.1480947476828012;
#     0.8217270194986074 0.1454170957775489;
#     0.83983286908078 0.1419155509783728;
#     0.8565459610027856 0.1388259526261586;
#     0.8586350974930362 0.1384140061791967;
#     0.8607242339832869 0.13779608650875386;
#     0.8662952646239553 0.13553038105046342;
#     0.8746518105849582 0.13223480947476826;
#     0.8837047353760447 0.12852729145211123;
#     0.8920612813370473 0.12523171987641607;
#     0.8990250696378829 0.1223480947476828;
#     0.9059888579387186 0.11946446961894952;
#     0.9129526462395544 0.11658084449021626;
#     0.91991643454039 0.113697219361483;
#     0.9317548746518106 0.10875386199794025;
#     0.9366295264623955 0.10628218331616889;
#     0.9456824512534819 0.10175077239958805;
#     0.953342618384401 0.09763130792996907;
#     0.9610027855153204 0.09351184346035016;
#     0.9658774373259055 0.09083419155509764;
#     0.9721448467966572 0.08692070030895985;
#     0.9770194986072421 0.08362512873326469;
#     0.9811977715877437 0.08074150360453139;
#     0.9860724233983288 0.07765190525231722;
#     0.9902506963788301 0.07456230690010296;
#     0.9937325905292481 0.07209062821833162;
#     0.998607242339833 0.06920700308959835;
#     1.0 0.06817713697219355
# ] .* rtip[1]
# chords = [AS.FM.linear(chords_raw[:,1], chords_raw[:,2], radii[1])]

# twists_raw = [
#     0.20689655          52.651941261622 # tacked on to reach hub radius
#     0.2317821454311707  52.651941261622;
#     0.254709079221614   51.51808027194226;
#     0.272959074351832   50.56838527075396;
#     0.2909986948984163  49.651357910884165;
#     0.3083545980481913  48.74133075986835;
#     0.3257105011979664  47.862415648203495;
#     0.3439604963281844  46.91427624898273;
#     0.3598963710384323  46.02058291862616;
#     0.3772522741882074  45.176095321212834;
#     0.3946081773379823  44.29411614506643;
#     0.4119640804877574  43.469411404997956;
#     0.4311433691656533  42.47811920314493;
#     0.4465005746342795  41.74112190590048;
#     0.4638564777840545  40.86998480407336;
#     0.4808617566277734  40.068849790785926;
#     0.4982176597775484  39.20549069879656;
#     0.516415061261858   38.32501998516417;
#     0.5345072754543507  37.502106544331056;
#     0.5527046769386602  36.63096944250393;
#     0.5707968911311531  35.77538836035231;
#     0.588152794280928   34.935363297876165;
#     0.6061924148275124  34.07589321080567;
#     0.625283908292265   33.22186773062159;
#     0.6433761224847576  32.47751218914967;
#     0.661415743031342   31.688821991602623;
#     0.6805072364960946  30.89079818225029;
#     0.6985994506885872  30.11299719847608;
#     0.7166390712351716  29.34375202552339;
#     0.7355727837621989  28.71373322866628;
#     0.7562420866042037  28.0557135963933;
#     0.7753861737148646  27.476251863481515;
#     0.7943198862418919  26.900679135488605;
#     0.8146210335625379  26.34766263602514;
#     0.8337651206731987  25.842869797555686;
#     0.8540662679938448  25.297631307929965;
#     0.8732103551045057  24.80061647929824;
#     0.8917759121101739  24.338602694936363;
#     0.9095657128386935  23.90808985041734;
#     0.9265052496250269  23.47835480688209;
#     0.9425024835888346  23.094315571143582;
#     0.9598583867386096  22.69180356204043;
#     0.9790550674951789  22.302424660818083;
#     1.0000000000000000  21.9829106386545
# ]
# twists = [AS.FM.linear(twists_raw[:,1], twists_raw[:,2], radii[1]) .* pi/180] # radians
# # set collective to 30 deg. at 70\% radius
# twists_07 = AS.FM.linear(radii[1], twists[1], 0.7 * rtip[1])
# twists[1] .+= 30.0 * pi/180 - twists_07

# airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "e212-il.dat")
# airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
# airfoilname = "eppler212_kevin"
# airfoilnames = [fill(airfoilname, length(radii[1]))]
# index = [1]
# positions = [[0.0, 0.35 * 2.58 / 2, 0.0]]
# orientations = [[-1.0, 0.0, 0.0]]
# spindirections = [true]

# function write_kevins_polars()
#     kevins_path = joinpath(AS.topdirectory, "data", "airfoil")
#     # epema_data = FileIO.load(joinpath(kevins_path, "epema_data_from_kevin.jld2"))
#     # af = epema_data[:"af"][1] # just one radial section will suffice as they are identical

#     epema_data = FileIO.load(joinpath(kevins_path, "E212_from_kevin.jld2"))
#     af = epema_data[:"NDtable"]
#     # extract alpha, Re, M numbers
#     alphas = af.var_input[1] .* pi/180 # to radians
#     Res = af.var_input[2]
#     Ms = af.var_input[3]
#     cls = af.response_values[1]
#     cds = af.response_values[2]
#     cms = af.response_values[3]
#     convs = af.response_values[4]
#     info = "eppler212 data extracted from moore2019multipropopt"
#     # name files
#     airfoilname = "eppler212_kevin"
#     filenames = AS.airfoilfilenames(airfoilname, Res, Ms; viternaextrapolation=false, rotationcorrection=false, aoaset=false, extension = ".txt")
#     filepaths = joinpath.(AS.topdirectory, "data", "airfoil", "polars", AS.TODAY, filenames)
#     # loop over each Re, M
#     for (M_i, M) in enumerate(Ms)
#         for (Re_i, Re) in enumerate(Res)
#             # extract alpha
#             # i_nonzero = findall((x) -> x != 0.0, cls[:, Re_i, M_i])
#             i_conv = Bool.(convs[:, Re_i, M_i])
#             alpha = alphas[i_conv]
#             cl = cls[i_conv, Re_i, M_i]
#             cd = cds[i_conv, Re_i, M_i]
#             # build ccblade object
#             airfoil = AS.CC.AlphaAF(alpha, cl, cd, info, Re, M)
#             AS.plotairfoil(airfoil, filenames[Re_i, M_i], airfoilname, Re, M; closefigure = true, radians = false)
#             # write files
#             if !isdir(splitdir(filepaths[Re_i, M_i])[1]); mkpath(splitdir(filepaths[Re_i, M_i])[1]); end
#             AS.CC.write_af(filepaths[Re_i, M_i], airfoil; radians = false)
#         end
#     end

#     return alphas, Res, Ms, filepaths
# end

# function write_kevins_polars_extrapolated()
#     kevins_path = joinpath(AS.topdirectory, "data", "airfoil")
#     epema_data = FileIO.load(joinpath(kevins_path, "epema_data_from_kevin.jld2"))
#     af = epema_data[:"af"][1] # just one radial section will suffice as they are identical
#     # extract alpha, Re, M numbers
#     alphas = af[1].var_input[1] .* pi/180 # to radians
#     Res = af[1].var_input[2]
#     Ms = af[1].var_input[3]
#     cls = af[1].spl_response.coefs[2:end-1, 2:end-1, 2:end-1]
#     cds = af[2].spl_response.coefs[2:end-1, 2:end-1, 2:end-1]
#     cms = af[3].spl_response.coefs[2:end-1, 2:end-1, 2:end-1]
#     convs = af[4].spl_response.coefs[2:end-1, 2:end-1, 2:end-1]
#     info = "extrapolated eppler212 data extracted from moore2019multipropopt"
#     # name files
#     airfoilname = "eppler212_kevin"
#     filenames = AS.airfoilfilenames(airfoilname, Res, Ms; viternaextrapolation=true, rotationcorrection=true, aoaset=false, extension = "_Kcorr.txt")
#     filepaths = joinpath.(AS.topdirectory, "data", "airfoil", "polars", AS.TODAY, filenames)
#     # loop over each Re, M
#     for (M_i, M) in enumerate(Ms)
#         for (Re_i, Re) in enumerate(Res)
#             # extract alpha
#             # i_nonzero = findall((x) -> x != 0.0, cls[:, Re_i, M_i])
#             i_conv = Bool.(round.(convs[:, Re_i, M_i], digits=0))
#             alpha = alphas[i_conv]
#             cl = cls[i_conv, Re_i, M_i]
#             cd = cds[i_conv, Re_i, M_i]
#             # build ccblade object
#             airfoil = AS.CC.AlphaAF(alpha, cl, cd, info, Re, M)
#             AS.plotairfoil(airfoil, filenames[Re_i, M_i], airfoilname, Re, M; closefigure = true, radians = true)
#             # write files
#             if !isdir(splitdir(filepaths[Re_i, M_i])[1]); mkpath(splitdir(filepaths[Re_i, M_i])[1]); end
#             AS.CC.write_af(filepaths[Re_i, M_i], airfoil; radians = false)
#         end
#     end

#     return alphas, Res, Ms, filepaths
# end

# # alphas, Res, Machs, filepaths = write_kevins_polars()
# # alphas, Res, Machs, filepaths = write_kevins_polars_extrapolated()

# polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", AS.TODAY)
# plotstepi = 1:length(Js)

# Res_list = [fill(Res, length(radii[1]))]
# Ms_list = [fill(Machs, length(radii[1]))]

# simulationdata = AS.rotor_sweep_template(Js, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, positions, orientations, spindirections, Res_list, Ms_list; polardirectory = polardirectory, closefigure=false)
# objective = AS.runsimulation!(simulationdata...)

# epema_data = [
#     0.39907904834996144 0.4526289577244005;
#     0.4973138910207212 0.5378398458549013;
#     0.600153491941673 0.6145449943665193;
#     0.6999232540291632 0.6710341111346976;
#     0.798158096699923 0.724330105648177;
#     0.8979278587874134 0.7520958181610362;
#     0.9976976208749039 0.751138126418576;
#     1.0990023023791249 0.6437990888457081;
#     1.2033768227168071 0.27476118939925875;
# ]

# fig = plt.figure("rotorsweep")
# axs = fig.get_axes()
# axs[3].scatter(epema_data[:,1], epema_data[:,2], label="Epema")
# axs[3].legend()
# savedirectory = joinpath(AS.topdirectory, "data", "plots", AS.TODAY)
# if !isdir(savedirectory); mkpath(savedirectory); end
# fig.savefig(joinpath(savedirectory, "epema_rotor_sweep.pdf"))

# # check Re
# J = 1.0
# vinf = 10.0
# D = 2 * rtip[1]
# n = vinf / J / D
# omega = n * 2 * pi
# Re = sqrt((rtip[1] * omega)^2 + vinf^2) * chords[1][10] / 1.81e-5
# println("Typical tip Re_c = $Re")
# # @test isapprox(objective, 0.0)

# # end






# # @testset "wings" begin
# # # validate PROWIM, props off:
# # alphas = range(-5, stop=12, length = 18) .* pi/180
# # wing_b = 640e-3 * 2
# # wing_TR = 1.0 # hershey bar
# # wing_c = 240e-3
# # wing_AR = wing_b / wing_c
# # wing_θroot = 0.0
# # wing_θtip = 0.0
# # simulationdata = AS.cl_alpha_sweep_template(alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip)
# # objective = AS.runsimulation!(simulationdata...)
# # alpha = [
# #     0;
# #     1.079545454545455;
# #     2.121212121212121;
# #     3.1818181818181825;
# #     4.242424242424243;
# #     5.303030303030304;
# #     6.344696969696968;
# #     7.386363636363636;
# #     8.446969696969695;
# #     9.469696969696969;
# # ]
# # CLs = [
# #     0;
# #     0.07431693989071053;
# #     0.14644808743169413;
# #     0.2163934426229509;
# #     0.2863387978142078;
# #     0.35409836065573785;
# #     0.41530054644808756;
# #     0.47650273224043715;
# #     0.5420765027322405;
# #     0.6032786885245902
# # ]
# # fig = plt.figure("clalphasweep")
# # axs = fig.get_axes()
# # axs[1].scatter(alpha, CLs, marker = "x", label="PROWIM")
# # axs[1].legend()

# # @test objective == 0

# # alphas = [0.0, 4.0, 10.0] .* pi/180
# # ploti = 1:length(alphas)
# # wing_b = 640e-3 * 2
# # wing_TR = 1.0 # hershey bar
# # wing_c = 240e-3
# # wing_AR = wing_b / wing_c
# # wing_θroot = 0.0
# # wing_θtip = 0.0
# # simulationdata = AS.lift_distribution_template(ploti, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip; surfacenames = ["PROWIM, w/o props"])
# # objective = AS.runsimulation!(simulationdata...)

# # y2b_cn_0deg_noprops = [
# #     0.09694654312188172 0.00459016393442635;
# #     0.17520171062009982 0.00721311475409836;
# #     0.2525958660014256 0.005901639344262133;
# #     0.31520456165359956 0.005901639344262355;
# #     0.37782466143977206 0.0006557377049181135;
# #     0.4091290092658591 0.0006557377049181135;
# #     0.5308624376336422 0.003278688524590123;
# #     0.5621667854597292 0.003278688524590123;
# #     0.6247811831789025 0.0006557377049181135;
# #     0.6865174625801855 0.0019672131147541183;
# #     0.8108652886671419 0.0019672131147541183;
# #     0.8717405559515325 -0.0006557377049178914;
# #     0.9186970776906629 -0.0006557377049181135
# # ]

# # y2b_cn_4deg_noprops = [
# #     0.09710049893086248 0.3337704918032788;
# #     0.17448610121168928 0.3363934426229509;
# #     0.25274982181040634 0.3350819672131151;
# #     0.3153670705630795 0.33114754098360666;
# #     0.37714041339985754 0.31540983606557405;
# #     0.40933143264433364 0.30754098360655724;
# #     0.5311190306486102 0.28524590163934427;
# #     0.5615424091233072 0.2904918032786885;
# #     0.6250292230933714 0.2865573770491804;
# #     0.6867883107626516 0.277377049180328;
# #     0.7485759087669281 0.25508196721311494;
# #     0.8112330719885961 0.23278688524590174;
# #     0.8730434782608698 0.20000000000000018;
# #     0.9192045616535995 0.16590163934426239;
# # ]

# # y2b_cn_10deg_noprops = [
# #     0.09708624376336422 0.740327868852459;
# #     0.1744803991446902 0.739016393442623;
# #     0.2518859586600142 0.7324590163934428;
# #     0.314520313613685 0.7206557377049181;
# #     0.3771746258018532 0.6996721311475411;
# #     0.4085217391304348 0.6800000000000002;
# #     0.5303292943692088 0.6485245901639345;
# #     0.5616136849607984 0.657704918032787;
# #     0.62424233784747 0.6485245901639345;
# #     0.686041339985745 0.6209836065573772;
# #     0.8114240912330721 0.5449180327868852;
# #     0.8724533143264435 0.47147540983606573;
# #     0.919552387740556 0.4059016393442625;
# # ]

# # prowim_liftdistribution = [
# #     y2b_cn_0deg_noprops,
# #     y2b_cn_4deg_noprops,
# #     y2b_cn_10deg_noprops
# # ]

# # fig = plt.figure("liftdistribution")
# # axs = fig.get_axes()
# # alpha_labels = [0,4,10]
# # for (idata, data) in enumerate(prowim_liftdistribution)
# #     cratio = idata / length(alpha_labels)
# #     axs[3].scatter(data[:,1], data[:,2], marker = "x", color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=L"PROWIM, \alpha = " * "$(alpha_labels[idata])" * L"^\circ")
# # end
# # axs[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))

# # @test objective == 0

# # end # test wings




# # @testset "wings_and_rotors" begin

# # # simulation controls
# # alphas = [0.0, 4.0, 10.0] .* pi/180
# # ploti = 1:length(alphas)
# # # wing definition
# # wing_b = 640e-3 * 2
# # wing_TR = 1.0 # hershey bar
# # wing_c = 240e-3
# # wing_AR = wing_b / wing_c
# # wing_θroot = 0.0
# # wing_θtip = 0.0
# # # environment
# # environment = AS.Environment()
# # ν = environment.ν
# # # freestream definition
# # Re_c = 0.8e6
# # vinf = Re_c * ν / wing_c
# # vinfs = fill(vinf, length(alphas))
# # # rotor definition
# # omegas = ones(1,length(alphas)) .* 1000
# # nblades = [4]
# # radii = [[0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-3/2]
# # rhub = [radii[1][1]]
# # rtip = [radii[1][end]]
# # chords = [[9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] * 1e-3]
# # twists = [[35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0]] # * pi/180 provide in degrees to match airfoil files

# # airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "naca4412.dat")
# # airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
# # airfoilname = "naca4412"
# # airfoilnames = [fill(airfoilname, length(radii[1]))]
# # index = [1]
# # rotor_X = [[-201.8e-3, 300e-3, 0.0]]
# # rotor_orientation = [[-1.0, 0.0, 0.0]]
# # spindirections = [true]

# # polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", "20210524")
# # plotstepi = 1:length(alphas)

# # simulationdata = AS.vlm_bem_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, rotor_X, rotor_orientation, spindirections, Res_list = [fill([5e4, 1e5, 1e6], length(radii))],
# #         polardirectory = polardirectory
# #     )
# # objective = AS.runsimulation!(simulationdata...)

# # end # test wings_and_rotors

# # # test rotor wake
# # parameters = simulationdata[2]
# # wakefun_aoa4 = parameters.wakefunctions[2]
# # x = range(-0.2, stop=1.0, length=25)
# # y = range(0.0, stop=wing_b/2, length=17)
# # z = range(-0.2, stop=0.2, length=13)
# # lx = length(x)
# # ly = length(y)
# # lz = length(z)
# # xs = [x[i] for i in 1:lx, j in 1:ly, k in 1:lz]
# # ys = [y[j] for i in 1:lx, j in 1:ly, k in 1:lz]
# # zs = [z[k] for i in 1:lx, j in 1:ly, k in 1:lz]
# # Vs = wakefun_aoa4.([[x[i], y[j], z[k]] for i in 1:lx, j in 1:ly, k in 1:lz])
# # # Vs ./ AS.LA.norm.(Vs)
# # us = [Vs[i,j,k][1] for i in 1:lx, j in 1:ly, k in 1:lz]
# # vs = [Vs[i,j,k][2] for i in 1:lx, j in 1:ly, k in 1:lz]
# # ws = [Vs[i,j,k][3] for i in 1:lx, j in 1:ly, k in 1:lz]

# # fig = plt.figure("test_wakefunction")
# # fig.clear()
# # ax = fig.add_subplot(111, projection="3d")
# # ax.quiver3D(xs, ys, zs, us, vs, ws, length=0.1)#, normalize=true)

# # test rotor wake
# parameters = simulationdata[2]
# wakefun_aoa4 = parameters.wakefunctions[2]
# x = range(-0.2, stop=1.0, length=7)
# y = range(0.0, stop=wing_b/2, length=5)
# z = range(-0.2, stop=0.2, length=5)
# lx = length(x)
# ly = length(y)
# lz = length(z)
# xs = [x[i] for i in 1:lx, j in 1:ly, k in 1:lz]
# ys = [y[j] for i in 1:lx, j in 1:ly, k in 1:lz]
# zs = [z[k] for i in 1:lx, j in 1:ly, k in 1:lz]
# Vs = wakefun_aoa4.([[x[i], y[j], z[k]] for i in 1:lx, j in 1:ly, k in 1:lz])
# # Vs ./ AS.LA.norm.(Vs)
# us = [Vs[i,j,k][1] for i in 1:lx, j in 1:ly, k in 1:lz]
# vs = [Vs[i,j,k][2] for i in 1:lx, j in 1:ly, k in 1:lz]
# ws = [Vs[i,j,k][3] for i in 1:lx, j in 1:ly, k in 1:lz]

# fig = plt.figure("test_wakefunction")
# fig.clear()
# ax = fig.add_subplot(111, projection="3d")
# ax.quiver3D(xs, ys, zs, us, vs, ws, length=0.1)#, normalize=true)


@testset "Epema Blown Wing Validation" begin

    include(joinpath(AS.topdirectory, "test", "EpemaData.jl"))

    wing = EpemaData.wing
    rotor = EpemaData.rotor
    setup = EpemaData.setup
    results = EpemaData.results

    Vinf = setup[:"Vinf"]
    vinfs = [Vinf]
    plotstepi = [1]
    alphas = [setup[:"wing_aoa"]]
    wing_b = wing[:"span"]
    wing_TR = wing["chord_tip"] / wing[:"chord_root"]
    wing_AR = wing["span"]^2 / wing[:"area"]
    wing_θroot = 0.0
    wing_θtip = 0.0
    rotor_omegas = [AS.get_omega(Vinf=setup[:"Vinf"], J=setup[:"J"], D=rotor[:"diameter"])]
    nblades = [rotor[:"Nblades"]]
    rhub = [rotor[:"radius_hub"] / 2]
    rtip = [rotor[:"radius"]]
    rotor_chord_data = rotor["r/R vs chord/R"]
    rotor_twist_data = rotor["r/R vs twist"]

    radii = [rotor_chord_data[:,1] * rtip[1]]
    rotor_chords = [rotor_chord_data[:,2] * rtip[1]]
    rotor_twists = [FM.linear(rotor_twist_data[:,1]*rtip[1], rotor_twist_data[:,2], radii[1])]
    airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "e212-il.csv")
    airfoilcontours = [fill(airfoilcontour, length(radii[1]))]
    airfoilname = "eppler212"
    airfoilnames = [fill(airfoilname, length(radii[1]))]
    index = [1]
    rotor_positions = [[0.0; rotor[:"y"]; 0.0]]
    rotor_orientations = [[-1.0; 0.0; 0.0]] # positive x downstream
    spindirections = [true]

    epema_chords = EpemaData.wing[:"chords"]
    xle = [0.0, 0.0, 0.0]
    yle = epema_chords[:,1]
    zle = [0.0, 0.0, 0.0]
    wing_chord = epema_chords[:,2]
    wing_twist = [0.0, 0.0, 0.0]
    wing_phi = [0.0, 0.0, 0.0]

    Res_list = [fill([EpemaData.setup[:"Re"]], length(radii))]
    surfacenames = ["epema wing"]
    polardirectory=joinpath(AS.topdirectory, "data","airfoil","polars","20210618")

    args = AS.vlm_bem_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip,
                            rotor_omegas, nblades, rhub, rtip, radii, rotor_chords, rotor_twists,
                            airfoilcontours, airfoilnames, index, rotor_positions, rotor_orientations,
                            spindirections;
                            Res_list,
                            surfacenames,
                            polardirectory,
                            # other wing keyword arguments
                            xle, yle, zle, wing_chord, wing_twist, wing_phi)

    outs = AS.runsimulation!(args...)

    aircraft = args[1]
    parameters = args[2]

    span_plot = aircraft.wingsystem.lifting_line_rs[1][2,2:end]
    cls_plot = parameters.cfs[1][3,:]
    wing_chords = FM.linear(epema_chords[:,1], epema_chords[:,2], range(epema_chords[1,1], stop=epema_chords[3,1], length=length(span_plot)))
    cls_plot = cls_plot .* wing_chords / EpemaData.wing[:"mac"]

    #! The values from Epema are normalized, I haven't normalized our results yet.
    figure()
    plot(span_plot, cls_plot, label="BEM+VLM")
    plot(EpemaData.results[:"lift distribution VLM"][:,1]*EpemaData.wing[:"span"]/2, EpemaData.results[:"lift distribution VLM"][:,2], label="Epema VLM model")
    scatter(EpemaData.results[:"normalized cl experimental"][:,1]*EpemaData.wing[:"span"]/2, EpemaData.results[:"normalized cl experimental"][:,2], label="Epema experimental")
    xlabel(L"y")
    ylabel(L"normalized\ c_l (c/c_{mac})")
    legend()

    # no tests for now - eventually would like to add some tests on cl values, as well as add results to the plot

end
