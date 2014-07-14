add_subdirectory(deferred_rendering)

set (files
  camera_noise_gaussian_fs.glsl
  camera_noise_gaussian_vs.glsl

  grid_fp.glsl
  grid_vp.glsl
  shadow_caster_fp.glsl
  shadow_caster_vp.glsl
  spot_shadow_receiver_fp.glsl
  spot_shadow_receiver_vp.glsl
  directional_shadow_receiver_fp.glsl
  directional_shadow_receiver_vp.glsl
  point_receiver_vp.glsl
  point_receiver_fp.glsl
  ambient_one_texture_vp.glsl
  blur.glsl
  perpixel_vp.glsl
  perpixel_fp.glsl
  depth_map.frag
  depth_map.vert
  depth_points_map.frag
  depth_points_map.vert
  laser_1st_pass.frag
  laser_1st_pass_dbg.frag
  laser_1st_pass.vert
  laser_2nd_pass.frag
  laser_2nd_pass.vert
  plain_color_vs.glsl
  plain_color_fs.glsl

  warp.vert
  warpWithChromeAb.frag
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/materials/programs/)
