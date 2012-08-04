SET (files 
  torso_lift_color.tif
  torso_lift.dae
  torso_lift_L.stl
  torso_lift_normals.tif
  torso_lift.stl
  torso.stl
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/pr2/torso_v0)
