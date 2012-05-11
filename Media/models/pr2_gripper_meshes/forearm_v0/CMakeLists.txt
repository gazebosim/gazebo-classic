SET (files 
 wrist_roll_L.stl
 wrist_normals.tif
 forearm.stl
 forearm.dae
 forearm.jpg
 wrist_color.tif
 forearm_normals.tif
 wrist_flex.stl
 wrist_flex.dae
 forearm_color.tif
 wrist_roll.stl
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/models/pr2_gripper_meshes/forearm_v0)


