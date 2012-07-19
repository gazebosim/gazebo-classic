set (files
  ambient_ps.glsl
  light_material_ps.glsl
  light_material_vs.glsl
  ShadowCaster_fp.glsl
  ShadowCaster_vp.glsl
  ssao_blurx.glsl
  ssao_blury.glsl
  ssao_fp.glsl
  ssao_vp.glsl
  vs.glsl
) 

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/materials/programs/deferred_shading)
