SET (files 
  base_color.tif
  base.dae
  base_L.stl
  base_normals.tif
  base.stl
  caster_L.stl
  caster.stl
  wheel_color.tif
  wheel.dae
  wheel_h_color.tif
  wheel_h.dae
  wheel_normals.tif
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/pr2/base_v0)
