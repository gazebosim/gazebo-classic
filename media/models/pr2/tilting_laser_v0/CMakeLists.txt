SET (files 
  hok_tilt.stl
  tilting_hokuyo_color.tif
  tilting_hokuyo.dae
  tilting_hokuyo_L.stl
  tilting_hokuyo_normals.tif
  tilting_hokuyo.stl
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/pr2/tilting_laser_v0)
