SET (files 
arm0_convex.stl
arm0.dae
arm1_convex.stl
arm1.dae
arm2_convex.stl
arm2.dae
arm3_convex.stl
arm3.dae
arm4_convex.stl
arm4.dae
arm5_convex.stl
arm5.dae
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/youbot/arm)

