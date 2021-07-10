AS = AircraftSystems
aircraft = simulationdata[1]
parameters = simulationdata[2]

struct TestParams
    us
    vs
    wakefunctions
    wakeshapefunctions
    axialinterpolation
    swirlinterpolation
    axialmultiplier
    swirlmultiplier
end

nsteps = length(parameters.us)
stepi = 5

testparams = TestParams(deepcopy(parameters.us), deepcopy(parameters.vs), Vector{Any}(nothing,nsteps), fill((Rtip, x) -> Rtip, length(aircraft.rotorsystem.index)), fill((rs, us, r, Rtip) -> FM.linear(rs, us, r), length(aircraft.rotorsystem.index)), fill((rs, vs, r, Rtip) -> FM.linear(rs, vs, r), length(aircraft.rotorsystem.index)), fill((distance2plane, Rtip) -> 2, length(aircraft.rotorsystem.index)), fill((distance2plane, Rtip) -> 1, length(aircraft.rotorsystem.index)))

for i=1:nsteps
    AS.solve_rotor_wake(aircraft, testparams, nothing, nothing, nothing, i, nothing)
end

wakefun_aoa4 = testparams.wakefunctions[2]
r = aircraft.rotorsystem.rotors[1].Rtip
# thetas = range(0, 2*pi, length=12)
# rs = range(r/10, 2*r, length=4)
# x = [r * cos(theta) for theta in thetas, r in rs]
# y = [r * cos(theta) for theta in thetas, r in rs]
# Vs = wakefun_aoa4.()
x = range(-r * 2.0, stop = r * 8.5, length=3)
y = range(-r * 2.5, stop = r * 2.5, length=35)
z = range(-r * 2.5, stop = r * 2.5, length=35)
lx = length(x)
ly = length(y)
lz = length(z)
xs = [x[i] for i in 1:lx, j in 1:ly, k in 1:lz]
ys = [y[j] for i in 1:lx, j in 1:ly, k in 1:lz]
zs = [z[k] for i in 1:lx, j in 1:ly, k in 1:lz]
Vs = wakefun_aoa4.([[x[i], y[j], z[k]] for i in 1:lx, j in 1:ly, k in 1:lz])
# Vs ./ AS.LA.norm.(Vs)
us = [Vs[i,j,k][1] for i in 1:lx, j in 1:ly, k in 1:lz]
vs = [Vs[i,j,k][2] for i in 1:lx, j in 1:ly, k in 1:lz]
ws = [Vs[i,j,k][3] for i in 1:lx, j in 1:ly, k in 1:lz]

fig = plt.figure("test_wakefunction")
fig.clear()
ax = fig.add_subplot(111, projection="3d")
ax.quiver3D(xs / r, ys / r, zs / r, us, vs, ws, length=0.01)#, normalize=true)

ax.set_xlabel(L"x/r")
ax.set_ylabel(L"y/r")
ax.set_zlabel(L"z/r")

fig2d = plt.figure("test_wakefunction_2d")
fig2d.clear()
ax2 = fig2d.add_subplot(111)
ax2.quiver(ys[1,:,:] / r, zs[1,:,:] / r, (vs[1,:,:] .+ 1e-10), (ws[1,:,:] .+ 1e-10))#, length=0.01, normalize=true)
thetas = range(0,2*pi,length=100)
rotor_y = cos.(thetas)
rotor_z = sin.(thetas)
ax2.plot(rotor_y, rotor_z, color="blue")
ax2.set_xlabel(L"x/r")
ax2.set_ylabel(L"y/r")

rs = range(r/200, 1.5*r, length=100)
xs_3 = vcat(-reverse(rs), rs)
Vs_3 = wakefun_aoa4.([[0.0, r, 0.0] for r in xs_3])
us = [v[1] for v in Vs_3]
vs = [v[2] for v in Vs_3]
ws = [v[3] for v in Vs_3]

fig3 = plt.figure("test_wakefunction_us_vs")
fig3.clear()
ax_u = fig3.add_subplot(311)
ax_v = fig3.add_subplot(312)
ax_w = fig3.add_subplot(313)
ax_u.plot(xs_3 ./ r, us)
ax_u.set_ylabel(L"u[m/s]")
ax_v.plot(xs_3 ./ r, vs)
ax_v.set_ylabel(L"v[m/s]")
ax_w.plot(xs_3 ./ r, ws)
ax_w.set_ylabel(L"w[m/s]")
ax_w.set_xlabel(L"y/R, x=z=0")
