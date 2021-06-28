parameters = simulationdata[2]
wakefun_aoa4 = parameters.wakefunctions[2]
x = range(-0.2, stop=1.0, length=25)
y = range(0.0, stop=wing_b/2, length=17)
z = range(-0.2, stop=0.2, length=13)
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
ax.quiver3D(xs, ys, zs, us, vs, ws, length=0.1)#, normalize=true)
