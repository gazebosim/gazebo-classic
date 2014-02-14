set (files 
  atlas_cage_and_camera_diffuse_flat.jpg
  atlas_DRC_1.png
  atlas_DRC_carbon_fiber.png
  atlas_DRC_dark_1.png
  drc_extremities_diffuse.jpg
  drc_labels_1.jpg
  drc_torso_head_diffuse.jpg
  atlas_cage_and_camera_diffuse_flat.jpg
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/)
