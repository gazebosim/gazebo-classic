set (files
  ambient_ps.glsl
  light_material_ps.glsl
  light_material_vs.glsl
  ShadowCaster_fp.glsl
  ShadowCaster_vp.glsl
  vs.glsl
) 

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/materials/programs/deferred_rendering/deferred_shading)
