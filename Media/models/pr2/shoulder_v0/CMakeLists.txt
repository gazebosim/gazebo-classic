set (files shoulder_lift_color.tif
           shoulder_lift.dae
           shoulder_lift_normals.tif
           shoulder_lift.stl
           shoulder_pan_color.tif
           shoulder_pan.dae
           shoulder_pan_normals.tif
           shoulder_pan.stl
           shoulder_yaw.stl
           upper_arm_roll_color.tif
           upper_arm_roll.dae
           upper_arm_roll_L.stl
           upper_arm_roll_normals.tif
           upper_arm_roll.stl
)


install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/models/pr2/shoulder_v0)
