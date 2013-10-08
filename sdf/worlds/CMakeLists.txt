set (files 
  actor.world
  actor_demo.world
  actor_circle.world
  two_actors.world
  camera.world
  gripper.world
  joint_damping_demo.world
  depth_camera.world
  empty.world
  force_torque_demo.world
  force_torque_demo2.world
  joints.world
  heightmap.world
  lights.world
  model_nesting.world
  pioneer2dx.world
  openal.world
  trimesh_test.world
  cart_test.world
  double_pendulums.world
  mud.world
  plugin.world
  pr2.world
  presentation.world
  projector.world
  ray_cpu.world
  rubble.world
  shapes.world
  weshare2012.world
  willowgarage.world)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/)
