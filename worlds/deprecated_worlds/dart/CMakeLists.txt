set (files
  colliding_pendulums.world
  colliding_pendulums_ode.world
  falling_box.world
  falling_box_ode.world
  falling_boxes.world
  falling_boxes_ode.world
  falling_shapes.world
  falling_shapes_ode.world
  double_pendulum.world
  double_pendulum_ode.world
  simple_pendulum.world
  simple_pendulum_ode.world
  simple_pendulum2.world
  simple_pendulum2_ode.world
  simple_pendulum2_bullet.world
  simple_pendulums.world
  simple_pendulums_ode.world
  pendulums.world
  pendulums_ode.world
  poses.world
  poses_ode.world
  shapes.world
  shapes_ode.world
  revolute_joint_test.world
  revolute_joints.world
  revolute_joints_ode.world
  sp_test.world
  sp_test_ode.world
  falling.world
  falling2.world
  falling_ode.world
  falling2_ode.world
  coll_filtering.world
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/dart)
