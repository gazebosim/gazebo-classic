set (files
  colliding_pendulums.world
  colliding_pendulums_ode.world
  falling_box.world
  falling_box_ode.world
  falling_boxes.world
  falling_boxes_ode.world
  double_pendulum.world
  double_pendulum_ode.world
  simple_pendulum.world
  simple_pendulum_ode.world
  pendulums.world
  pendulums_ode.world
  poses.world
  poses_ode.world
  shapes.world
  shapes_ode.world
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/rtql8)
