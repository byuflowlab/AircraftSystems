# Integrate cl distributions

aircraft_CL = data_PROWIM_CL_alpha[1]
parameters_CL = data_PROWIM_CL_alpha[2]

lifting_line_rs_CL = aircraft_CL.wing_system.lifting_line_rs[1]
lifting_line_xs_CL = lifting_line_rs_CL[1,:]
lifting_line_ys_CL = lifting_line_rs_CL[2,:]
lifting_line_zs_CL = lifting_line_rs_CL[3,:]

cpxs_CL = vcat([panel.rtl[1] for panel in aircraft_CL.wing_system.system.surfaces[1]][1,:], aircraft_CL.wing_system.system.surfaces[1][end].rtr[1])
cpys_CL = vcat([panel.rtl[2] for panel in aircraft_CL.wing_system.system.surfaces[1]][1,:], aircraft_CL.wing_system.system.surfaces[1][end].rtr[2])
cpzs_CL = vcat([panel.rtl[3] for panel in aircraft_CL.wing_system.system.surfaces[1]][1,:], aircraft_CL.wing_system.system.surfaces[1][end].rtr[3])

aircraft_cl = data_PROWIM_lift_distribution[1]
parameters_cl = data_PROWIM_lift_distribution[2]

lifting_line_rs_cl = aircraft_cl.wing_system.lifting_line_rs[1]
lifting_line_xs_cl = lifting_line_rs_cl[1,:]
lifting_line_ys_cl = lifting_line_rs_cl[2,:]
lifting_line_zs_cl = lifting_line_rs_cl[3,:]

cpxs_cl = vcat([panel.rtl[1] for panel in aircraft_cl.wing_system.system.surfaces[1]][1,:], aircraft_cl.wing_system.system.surfaces[1][end].rtr[1])
cpys_cl = vcat([panel.rtl[2] for panel in aircraft_cl.wing_system.system.surfaces[1]][1,:], aircraft_cl.wing_system.system.surfaces[1][end].rtr[2])
cpzs_cl = vcat([panel.rtl[3] for panel in aircraft_cl.wing_system.system.surfaces[1]][1,:], aircraft_cl.wing_system.system.surfaces[1][end].rtr[3])

fig_check_geometry_x = plt.figure("prowim_geometry_x")
fig_check_geometry_x.clear()
fig_check_geometry_x.add_subplot(111)

ax_x = fig_check_geometry_x.get_axes()[1]
ax_x.plot(1:length(lifting_line_xs_CL), lifting_line_xs_CL, "+", label="CL-alpha, lifting_line_xs")
ax_x.plot(1:length(cpxs_CL), cpxs_CL, "x", label="CL-alpha, bound vortex")
ax_x.plot(1:length(lifting_line_xs_cl), lifting_line_xs_cl, "-", label="cl dist, lifting_line_xs")
ax_x.plot(1:length(cpxs_cl), cpxs_cl, "--", label="cl dist, bound vortex")
ax_x.set_ylabel(LS.L"x")
ax_x.set_xlabel(LS.L"i")
ax_x.legend(loc="upper left", bbox_to_anchor=(1.01,1))

fig_check_geometry_y = plt.figure("prowim_geometry_y")
fig_check_geometry_y.clear()
fig_check_geometry_y.add_subplot(111)

ax_y = fig_check_geometry_y.get_axes()[1]
ax_y.plot(1:length(lifting_line_ys_CL), lifting_line_ys_CL, "+", label="CL-alpha, lifting_line_ys")
ax_y.plot(1:length(cpys_CL), cpys_CL, "x", label="CL-alpha, bound vortex")
ax_y.plot(1:length(lifting_line_ys_cl), lifting_line_ys_cl, "-", label="cl dist, lifting_line_ys")
ax_y.plot(1:length(cpys_cl), cpys_cl, "--", label="cl dist, bound vortex")
ax_y.set_ylabel(LS.L"y")
ax_y.set_xlabel(LS.L"i")
ax_y.legend(loc="upper left", bbox_to_anchor=(1.01,1))

fig_check_geometry_z = plt.figure("prowim_geometry_z")
fig_check_geometry_z.clear()
fig_check_geometry_z.add_subplot(111)

ax_z = fig_check_geometry_z.get_axes()[1]
ax_z.plot(1:length(lifting_line_zs_CL), lifting_line_zs_CL, "+", label="CL-alpha, lifting_line_zs")
ax_z.plot(1:length(cpxs_CL), cpzs_CL, "x", label="CL-alpha, bound vortex")
ax_z.plot(1:length(lifting_line_zs_cl), lifting_line_zs_cl, "-", label="cl dist, lifting_line_zs")
ax_z.plot(1:length(cpxs_cl), cpzs_cl, "--", label="cl dist, bound vortex")
ax_z.set_ylabel(LS.L"z")
ax_z.set_xlabel(LS.L"i")
ax_z.legend(loc="upper left", bbox_to_anchor=(1.01,1))

savename_x = "prowim_check_geometry_x.pdf"
savename_y = "prowim_check_geometry_y.pdf"
savename_z = "prowim_check_geometry_z.pdf"
fig_check_geometry_x.savefig(joinpath(plot_directory, savename_x), bbox_inches="tight")
fig_check_geometry_y.savefig(joinpath(plot_directory, savename_y), bbox_inches="tight")
fig_check_geometry_z.savefig(joinpath(plot_directory, savename_z), bbox_inches="tight")

# properties_cl = VL.get_surface_properties(aircraft_cl.wing_system.system)

# vtk_directory = joinpath(AS.topdirectory, "test", "data", "vtk", "prowim")
# # if !isdir(vtk_directory); mkpath(vtk_directory); end
# # VL.write_vtk(joinpath(vtk_directory, "prowim_wing"), aircraft_cl.wing_system.surfaces, properties)
# VL.write_vtk("prowim_wing", aircraft_cl.wing_system.system.surfaces, properties; symmetric = false)
