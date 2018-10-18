SET (files 
  chair.dae
  chair.stl
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/models/chair3/model)
