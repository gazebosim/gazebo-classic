add_subdirectory(deferred_shading)
add_subdirectory(deferred_lighting)

set (files
  ssao_blurx.glsl
  ssao_blury.glsl
  ssao_fp.glsl
  ssao_vp.glsl
  vpl_fp.glsl
  vpl_lbuffer_fp.glsl
  vpl_vp.glsl
  RSM_vp.glsl
  RSM_fp.glsl
) 

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/materials/programs/deferred_rendering)
