set (models
  pioneer2at.model
  pr2.model
  pr2_visual_only.model
  pr2_gripper.model
  box.model
  table.model
  bookshelf.model
  bowl.model
  saucepan.model
  female_standing.model
  cylinder.model
  cube_20k.model
  youbot.model
  youbot_static.model
  quadrotor.model
  weshare2012_apartment.model
  weshare2012_pioneer.model
  weshare2012_bookshelf.model
  kitchen.model
  hokuyo.model
  camera.model
  simple_gripper.model
  simple_arm.model
  14_4v_cordless_drill.model
  coke_can.model
  car.model
  cabinet.model
  actor_circle.model
  actor_demo.model
  actor.model
  japanese_restaurant.model
  multi_visuals.model
  ragdoll.model
)

install(FILES ${models} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/)
