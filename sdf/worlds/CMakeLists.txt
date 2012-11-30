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
  trimesh_test.world
  cart_test.world
  double_pendulums.world
  plugin.world
  pr2.world
  presentation.world
  projector.world
  rubble.world
  shapes.world
  weshare2012.world
  willowgarage.world)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/)
