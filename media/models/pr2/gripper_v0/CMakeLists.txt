SET (files 
  finger_H_Color_100430.tif
  finger_H_UV_100430.dae
  fingertip_H_Color_100430.tif
  finger_tip_H_UV_100430.dae
  finger_tip_l.stl
  finger_tip_pad2_l.stl
  finger_tip_pad2_r.stl
  finger_tip_r.stl
  float_H_Color_100430.tif
  float_H_UV_100430.dae
  gripper_palm_color.tif
  gripper_palm.dae
  gripper_palm_normals.tif
  gripper_palm.stl
  l_finger_color.tif
  l_finger.dae
  l_finger_normals.tif
  l_finger.stl
  l_finger_tip_color.tif
  l_finger_tip.dae
  l_finger_tip_normals.tif
  l_finger_tip.stl
  l_float_color.tif
  l_float.dae
  l_floating.stl
  l_float_normals.tif
  palm_H_Color_100430.tif
  palm_H_UV_100430.dae
  upper_finger_l.stl
  upper_finger_r.stl
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/pr2/gripper_v0)
