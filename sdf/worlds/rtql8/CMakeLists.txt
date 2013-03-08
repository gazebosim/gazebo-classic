set (files
  collision1_rtql8.world
  double_pendulum_rtql8.world
  double_pendulum_ode.world
  simple_pendulum_rtql8.world
  simple_pendulum_ode.world
  simple_pendulums_rtql8.world
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/rtql8)
