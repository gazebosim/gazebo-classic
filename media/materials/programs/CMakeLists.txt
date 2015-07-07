add_subdirectory(deferred_rendering)

set (files
ambient_one_texture_vp.glsl
blur.glsl
camera_distortion_map_fs.glsl
camera_distortion_map_vs.glsl
camera_noise_gaussian_fs.glsl
camera_noise_gaussian_vs.glsl
depth_map.frag
depth_map.vert
depth_points_map.frag
depth_points_map.vert
directional_shadow_receiver_fp.glsl
directional_shadow_receiver_vp.glsl
grid_fp.glsl
grid_vp.glsl
laser_1st_pass_dbg.frag
laser_1st_pass.frag
laser_1st_pass.vert
laser_2nd_pass.frag
laser_2nd_pass.vert
oculus.cg
OculusDistortionFragment.glsl
OculusDistortionVertex.glsl
perpixel_fp.glsl
perpixel_vp.glsl
plain_color_fs.glsl
plain_color_vs.glsl
point_receiver_fp.glsl
point_receiver_vp.glsl
projector.frag
projector.vert
shadow_caster_fp.glsl
shadow_caster_vp.glsl
spot_shadow_receiver_fp.glsl
spot_shadow_receiver_vp.glsl
wide_lens_map_fp.glsl
wide_lens_map_vs.glsl
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/materials/programs/)

