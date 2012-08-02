SET (files 
hokuyo_convex.stl
hokuyo.dae
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/hokuyo)
