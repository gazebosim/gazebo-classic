SET (files 
  finger_H_UV_100430.dae
  finger_tip_H_UV_100430.dae
  float_H_UV_100430.dae
  gripper_palm.dae
  l_finger.dae
  l_finger_tip.dae
  l_float.dae
  palm_H_UV_100430.dae
  finger_H_Color_100430.tif
  fingertip_H_Color_100430.tif
  float_H_Color_100430.tif
  gripper_palm_color.tif
  gripper_palm_normals.tif
  l_finger_color.tif
  l_finger_normals.tif
  l_finger_tip_color.tif
  l_finger_tip_normals.tif
  l_float_color.tif
  l_float_normals.tif
  palm_H_Color_100430.tif
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/models/pr2_gripper)


