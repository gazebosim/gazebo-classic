SET (files 
finger_convex.stl
finger.stl
palm_convex.stl
palm.stl
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/youbot/gripper)
