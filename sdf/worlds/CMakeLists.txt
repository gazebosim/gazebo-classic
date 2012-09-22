set (files 
  actor.world
  actor_demo.world
  actor_circle.world
  two_actors.world
  camera.world
  depth_camera.world
  empty.world
  joints.world
  heightmap.world
  lights.world
  model_nesting.world
  pioneer2dx.world
  drop_test.world
  trimesh_test.world
  collision_test.world
  simple_pendulums.world
  cart_test.world
  double_pendulums.world
  plugin.world
  pr2.world
  presentation.world
  projector.world
  shapes.world
  weshare2012.world
  willowgarage.world)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/)
