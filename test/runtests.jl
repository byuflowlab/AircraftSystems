import AircraftSystems
AS = AircraftSystems
import FileIO
FM = AS.FM
using LaTeXStrings
plt = AS.plt
plt.pygui(false)
VL = AS.VL
using Test

# topdirectory = normpath(joinpath(@__DIR__,".."))

# @testset "rotors" begin

# # test rotor_sweep_template
nJs = 27
Js = ones(3,nJs) .* range(0.01, stop=2.0, length=nJs)'
# omegas = fill(50.0, length(Js))
# nblades = 3
# radii = [0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-3/2
# rhub = radii[1]
# rtip = radii[end]
# chords = [9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] * 1e-3
# twists = [35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0] # * pi/180 provide in degrees to match airfoil files
# airfoilcontour = joinpath(AS.topdirectory, "data", "airfoil", "contours", "naca4412.dat")
# airfoilcontours = fill(airfoilcontour, length(radii))
# airfoilname = "naca4412"
# airfoilnames = fill(airfoilname, length(radii))
# polardirectory = joinpath(AS.topdirectory, "data", "airfoil", "polars", "20210524")


# # PROWIM propeller
# omegas = ones(1,length(Js)) .* 5000 * 2 * pi / 60 # rad/s
# nblades = [4]
# radii = [[0.148, 0.254237, 0.381356, 0.508475, 0.635593, 0.762712, 0.889831, 1.0] .* 236e-3/2]
# rhub = [radii[1][1]]
# rtip = [radii[1][end]]
# chords = [[9.88, 11.88, 15.59, 18.81, 19.55, 18.32, 13.96, 0.01] .* 1e-3]
# twists = [[35.0, 32.5, 26.5, 23.5, 19, 16.5, 14.0, 10.0]] * pi/180 # provide in degrees to match airfoil files

include("epema_rotor_sweep.jl")

# end






# @testset "wings" begin
# # validate PROWIM, props off:
# alphas = range(-5, stop=12, length = 18) .* pi/180
# wing_b = 640e-3 * 2
# wing_TR = 1.0 # hershey bar
# wing_c = 240e-3
# wing_AR = wing_b / wing_c
# wing_θroot = 0.0
# wing_θtip = 0.0
# simulationdata = AS.cl_alpha_sweep_template(alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip)
# objective = AS.runsimulation!(simulationdata...)
# alpha = [
#     0;
#     1.079545454545455;
#     2.121212121212121;
#     3.1818181818181825;
#     4.242424242424243;
#     5.303030303030304;
#     6.344696969696968;
#     7.386363636363636;
#     8.446969696969695;
#     9.469696969696969;
# ]
# CLs = [
#     0;
#     0.07431693989071053;
#     0.14644808743169413;
#     0.2163934426229509;
#     0.2863387978142078;
#     0.35409836065573785;
#     0.41530054644808756;
#     0.47650273224043715;
#     0.5420765027322405;
#     0.6032786885245902
# ]
# fig = plt.figure("clalphasweep")
# axs = fig.get_axes()
# axs[1].scatter(alpha, CLs, marker = "x", label="PROWIM")
# axs[1].legend()

# @test objective == 0

# alphas = [0.0, 4.0, 10.0] .* pi/180
# ploti = 1:length(alphas)
# wing_b = 640e-3 * 2
# wing_TR = 1.0 # hershey bar
# wing_c = 240e-3
# wing_AR = wing_b / wing_c
# wing_θroot = 0.0
# wing_θtip = 0.0
# simulationdata = AS.lift_distribution_template(ploti, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip; surfacenames = ["PROWIM, w/o props"])
# objective = AS.runsimulation!(simulationdata...)

# y2b_cn_0deg_noprops = [
#     0.09694654312188172 0.00459016393442635;
#     0.17520171062009982 0.00721311475409836;
#     0.2525958660014256 0.005901639344262133;
#     0.31520456165359956 0.005901639344262355;
#     0.37782466143977206 0.0006557377049181135;
#     0.4091290092658591 0.0006557377049181135;
#     0.5308624376336422 0.003278688524590123;
#     0.5621667854597292 0.003278688524590123;
#     0.6247811831789025 0.0006557377049181135;
#     0.6865174625801855 0.0019672131147541183;
#     0.8108652886671419 0.0019672131147541183;
#     0.8717405559515325 -0.0006557377049178914;
#     0.9186970776906629 -0.0006557377049181135
# ]

# y2b_cn_4deg_noprops = [
#     0.09710049893086248 0.3337704918032788;
#     0.17448610121168928 0.3363934426229509;
#     0.25274982181040634 0.3350819672131151;
#     0.3153670705630795 0.33114754098360666;
#     0.37714041339985754 0.31540983606557405;
#     0.40933143264433364 0.30754098360655724;
#     0.5311190306486102 0.28524590163934427;
#     0.5615424091233072 0.2904918032786885;
#     0.6250292230933714 0.2865573770491804;
#     0.6867883107626516 0.277377049180328;
#     0.7485759087669281 0.25508196721311494;
#     0.8112330719885961 0.23278688524590174;
#     0.8730434782608698 0.20000000000000018;
#     0.9192045616535995 0.16590163934426239;
# ]

# y2b_cn_10deg_noprops = [
#     0.09708624376336422 0.740327868852459;
#     0.1744803991446902 0.739016393442623;
#     0.2518859586600142 0.7324590163934428;
#     0.314520313613685 0.7206557377049181;
#     0.3771746258018532 0.6996721311475411;
#     0.4085217391304348 0.6800000000000002;
#     0.5303292943692088 0.6485245901639345;
#     0.5616136849607984 0.657704918032787;
#     0.62424233784747 0.6485245901639345;
#     0.686041339985745 0.6209836065573772;
#     0.8114240912330721 0.5449180327868852;
#     0.8724533143264435 0.47147540983606573;
#     0.919552387740556 0.4059016393442625;
# ]

# prowim_liftdistribution = [
#     y2b_cn_0deg_noprops,
#     y2b_cn_4deg_noprops,
#     y2b_cn_10deg_noprops
# ]

# fig = plt.figure("liftdistribution")
# axs = fig.get_axes()
# alpha_labels = [0,4,10]
# for (idata, data) in enumerate(prowim_liftdistribution)
#     cratio = idata / length(alpha_labels)
#     axs[3].scatter(data[:,1], data[:,2], marker = "x", color = (0.05, 0.85-cratio*0.7, 0.15 + 0.75 * cratio), label=L"PROWIM, \alpha = " * "$(alpha_labels[idata])" * L"^\circ")
# end
# axs[3].legend(loc="upper left", bbox_to_anchor=(1.01,1))

# @test objective == 0

# end # test wings




# @testset "wings_and_rotors" begin

# # simulation controls
# alphas = [0.0, 4.0, 10.0] .* pi/180
# ploti = 1:length(alphas)
# # wing definition
# wing_b = 640e-3 * 2
# wing_TR = 1.0 # hershey bar
# wing_c = 240e-3
# wing_AR = wing_b / wing_c
# wing_θroot = 0.0
# wing_θtip = 0.0
# # environment
# environment = AS.Environment()
# ν = environment.ν
# # freestream definition
# Re_c = 0.8e6
# vinf = Re_c * ν / wing_c
# vinfs = fill(vinf, length(alphas))
# # rotor definition
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

# simulationdata = AS.vlm_bem_template(vinfs, plotstepi, alphas, wing_b, wing_TR, wing_AR, wing_θroot, wing_θtip, omegas, nblades, rhub, rtip, radii, chords, twists, airfoilcontours, airfoilnames, index, rotor_X, rotor_orientation, spindirections, Res_list = [fill([5e4, 1e5, 1e6], length(radii))],
#         polardirectory = polardirectory
#     )
# objective = AS.runsimulation!(simulationdata...)


# # end # test wings_and_rotors

# # test rotor wake
# parameters = simulationdata[2]
# wakefun_aoa4 = parameters.wakefunctions[2]
# x = range(-0.2, stop=1.0, length=25)
# y = range(0.0, stop=wing_b/2, length=17)
# z = range(-0.2, stop=0.2, length=13)
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
