add_subdirectory(textures)
set (files willowgarage_visual.dae willowgarage_collision.dae)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/willowgarage)
