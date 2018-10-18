add_subdirectory(images)
add_subdirectory(models)

SET (files 
  textures.txt
  doc.kml
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/models/chair3)
