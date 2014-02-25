set (files

#BEGIN_CHECK_SDF
  blank.world
  camera.world
  gripper.world
  joint_damping_demo.world
  depth_camera.world
  empty.world
  empty_sky.world
  force_torque_demo.world
  friction_demo.world
  joints.world
  heightmap.world
  lights.world
  model_nesting.world
  pioneer2dx.world
  openal.world
  cart_demo.world
  mud.world
  plugin.world
  pr2.world
  presentation.world
  projector.world
  ray_cpu.world
  robocup_3Dsim.world
  robocup09_spl_field.world
  robocup14_spl_field.world
  rubble.world
  shapes.world
  sphere_atlas_demo.world
  willowgarage.world
#END_CHECK_SDF

)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/)
