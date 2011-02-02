set (files forearm_color.tif
           forearm.dae
           forearm_normals.tif
           forearm.stl
           wrist_color.tif
           wrist_flex.dae
           wrist_flex.stl
           wrist_normals.tif
           wrist_roll_L.stl
           wrist_roll.stl
)


install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/models/pr2/forearm_v0)
