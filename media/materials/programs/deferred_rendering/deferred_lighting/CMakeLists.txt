set (files
  ambient_ps.glsl
) 

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/materials/programs/deferred_rendering/deferred_lighting)
