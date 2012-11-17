set (files 
  actor.world
  actor_demo.world
  actor_circle.world
  camera.world
  cart_test.world
  collision_test.world
  depth_camera.world
  double_pendulums.world
  drop_test.world
  empty.world
  joints.world
  heightmap.world
  lights.world
  model_nesting.world
  multi_collision_test.world
  pioneer2dx.world
  plugin.world
  pr2.world
  presentation.world
  projector.world
  rubble.world
  shapes.world
  simple_pendulums.world
  spring_test.world
  two_actors.world
  trimesh_test.world
  weshare2012.world
  willowgarage.world)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/)
