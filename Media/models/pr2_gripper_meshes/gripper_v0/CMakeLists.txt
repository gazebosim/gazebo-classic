SET (files 
 l_finger_normals.tif
 fingertip_H_Color_100430.tif
 finger_tip_pad2_l.stl
 palm_H_UV_100430.dae
 finger_H_UV_100430.dae
 l_floating.stl
 gripper_palm_normals.tif
 l_float.dae
 float_H_UV_100430.dae
 palm_H_Color_100430.tif
 gripper_palm.stl
 l_float_normals.tif
 l_finger.stl
 gripper_palm_color.tif
 upper_finger_l.stl
 finger_tip_l.stl
 l_finger_tip.stl
 l_finger_tip_color.tif
 finger_tip_H_UV_100430.dae
 l_finger.dae
 upper_finger_r.stl
 float_H_Color_100430.tif
 l_finger_tip_normals.tif
 finger_tip_r.stl
 gripper_palm.dae
 l_finger_color.tif
 finger_H_Color_100430.tif
 l_finger_tip.dae
 finger_tip_pad2_r.stl
 l_float_color.tif
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/models/pr2_gripper_meshes/gripper_v0)


