set (files 
  actor.world
  camera.world
  depth_camera.world
  empty.world
  joints.world
  kitchen.world
  lights.world
  model_nesting.world
  pioneer2dx.world
  plugin.world
  pr2.world
  presentation.world
  shapes.world
  weshare2012.world
  willowgarage.world)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/)
