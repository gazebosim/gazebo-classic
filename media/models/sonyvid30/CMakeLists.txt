SET (files sonyvid30_bottom.mesh
           sonyvid30_mid.mesh
           sonyvid30_top.mesh
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/sonyvid30)
