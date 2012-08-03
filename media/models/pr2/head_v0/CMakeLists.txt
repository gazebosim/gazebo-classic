SET (files 
  head_pan_color.tif
  head_pan.dae
  head_pan_L.stl
  head_pan_normals.tif
  head_pan.stl
  head_tilt_color_red.tif
  head_tilt_color.tif
  head_tilt_color_yellow.tif
  head_tilt.dae
  head_tilt_green.tif
  head_tilt_L.stl
  head_tilt_normals.tif
  head_tilt.stl
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/pr2/head_v0)
