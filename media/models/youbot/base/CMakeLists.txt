SET (files 
base_convex.stl
base.dae
back-left_wheel.dae
back-right_wheel.dae
front-left_wheel.dae
front-right_wheel.dae
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/youbot/base)

