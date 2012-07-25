add_subdirectory(bandit)
add_subdirectory(blender)
add_subdirectory(epuck)
add_subdirectory(japanese_restaurant)
add_subdirectory(PioneerArm)
add_subdirectory(pioneer2dx)
add_subdirectory(pioneer2at)
add_subdirectory(sonyvid30)
add_subdirectory(rovio)
add_subdirectory(pr2)
add_subdirectory(collada_quad_test)
add_subdirectory(chair3)
add_subdirectory(wg)
add_subdirectory(frypan)
add_subdirectory(people)
add_subdirectory(youbot)
add_subdirectory(hokuyo)
add_subdirectory(quadrotor)
add_subdirectory(willowgarage)
add_subdirectory(kitchen)

set (files 
  bowl.dae
  sub_16.dae
  moonwalk.dae
  run.dae
  sit_down.dae
  stand_up.dae
  sitting.dae
  walk.dae
  stand.dae
  talk_a.dae
  talk_b.dae
  coke_can.dae
  cube_20k.stl
  14_4v_cordless_drill.dae
  14_4v_cordless_drill.jpg
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/models)
