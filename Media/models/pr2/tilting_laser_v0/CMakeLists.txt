set (files hok_tilt.stl
           tilting_hokuyo_color.tif
           tilting_hokuyo.dae
           tilting_hokuyo_L.stl
           tilting_hokuyo_normals.tif
           tilting_hokuyo.stl
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/models/pr2/tilting_laser_v0)
