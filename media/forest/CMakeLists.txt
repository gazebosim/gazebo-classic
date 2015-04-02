FILE(GLOB files *.png *.jpg *.material *.blend *.mesh)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/forest/)
