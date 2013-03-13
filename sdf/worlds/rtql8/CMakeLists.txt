set (files
  colliding_pendulums_rtql8.world
  colliding_pendulums_ode.world
  collision1_rtql8.world
  collision2_rtql8.world
  collision1_ode.world
  double_pendulum_rtql8.world
  double_pendulum_ode.world
  simple_pendulum_rtql8.world
  simple_pendulum_ode.world
  pendulums_rtql8.world
  pendulums_ode.world
  poses_rtql8.world
  poses_ode.world
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/rtql8)
