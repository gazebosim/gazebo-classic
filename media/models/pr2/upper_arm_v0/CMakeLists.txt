SET (files 
  elbow_flex_color.tif
  elbow_flex.dae
  elbow_flex_normals.tif
  elbow_flex.stl
  forearm_roll_L.stl
  forearm_roll.stl
  upper_arm_color.tif
  upper_arm.dae
  upper_arm.jpg
  upper_arm_normals.tif
  upper_arm.stl
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/pr2/upper_arm_v0)
