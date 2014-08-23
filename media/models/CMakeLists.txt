set (files
	moonwalk.dae
	run.dae
	sit_down.dae
	sitting.dae
	stand_up.dae
	stand.dae
	sub_16.dae
	talk_a.dae
	talk_b.dae
	walk.dae
  box.stl
  box45.stl
  cone.stl
  cylinder.stl
  mug.stl
  sphere.stl
  thick_mug.stl
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/)
