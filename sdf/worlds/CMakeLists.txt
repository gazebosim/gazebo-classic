set (files 
  actor.world
  actor_demo.world
  actor_circle.world
  two_actors.world
  camera.world
  joint_damping_demo.world
  depth_camera.world
  double_pendulum_rtql8.world
  double_pendulum_ode.world
  empty.world
  force_torque_demo.world
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
  simple_pendulum_rtql8.world
  simple_pendulum_ode.world
  simple_pendulums_rtql8.world
  weshare2012.world
  willowgarage.world)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/)
