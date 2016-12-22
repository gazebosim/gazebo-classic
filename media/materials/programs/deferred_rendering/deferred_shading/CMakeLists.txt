set (files
  ambient_ps.glsl
  light_material_ps.glsl
  light_material_vs.glsl
  ShadowCaster_fp.glsl
  ShadowCaster_vp.glsl
  vs.glsl
) 

INSTALL(FILES ${files} DESTINATION ${MEDIA_ROOT_DESTINATION}/materials/programs/deferred_rendering/deferred_shading)
