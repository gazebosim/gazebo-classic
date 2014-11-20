include_directories (
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

link_directories(
  ${ogre_library_dirs}
  ${Boost_LIBRARY_DIRS}
  ${ODE_LIBRARY_DIRS}
)

# Disable the player test for now, until we figure out a better way to test them
#if (INCLUDE_PLAYER)
#  add_subdirectory(player)
#endif()

if(HAVE_BULLET)
  include_directories ( ${BULLET_INCLUDE_DIRS} )
  link_directories ( ${BULLET_LIBRARY_DIRS} )
endif()
if(HAVE_DART)
  include_directories ( ${DART_INCLUDE_DIRS} )
  link_directories ( ${DART_LIBRARY_DIRS} )
endif()

set(tests
  aero_plugin.cc
  bandwidth.cc
  concave_mesh.cc
  contact_sensor.cc
  dem.cc
  factory.cc
  file_handling.cc
  gripper.cc
  gz_joint.cc
  gz_log.cc
  gz_model.cc
  gz_physics.cc
  gz_world.cc
  imu.cc
  joint_controller.cc
  joint_force_torque.cc
  joint_gearbox.cc
  joint_revolute.cc
  joint_screw.cc
  joint_set_position_test.cc
  joint_spawn.cc
  joint_test.cc
  joint_universal.cc
  laser.cc
  model_database.cc
  noise.cc
  nondefault_world.cc
  physics.cc
  physics_basic_controller_response.cc
  physics_friction.cc
  physics_inertia_ratio.cc
  physics_link.cc
  physics_msgs.cc
  physics_thread_safe.cc
  pioneer2dx.cc
  plugin.cc
  server_fixture.cc
  speed.cc
  stress_spawn_models.cc
  #state_log.cc
  surface_properties.cc
  transceiver.cc
  transport.cc
  world.cc
  world_entity_below_point.cc
  worlds_installed.cc
  )

if (MANPAGES_SUPPORT)
  set (tests ${tests}
	      manpages.cc)
endif()

gz_build_tests(${tests})

if (ENABLE_TESTS_COMPILATION)
  # Increase timeout, to account for model download time.
  set_tests_properties(${TEST_TYPE}_factory PROPERTIES TIMEOUT 500)
  set_tests_properties(${TEST_TYPE}_physics PROPERTIES TIMEOUT 500)

  # Add plugin dependency
  add_dependencies(${TEST_TYPE}_joint_test SpringTestPlugin)
endif()

set(display_tests
)

# Build the display tests (need extra sources to compile)
gz_build_display_tests(${display_tests})

set(dri_tests
  camera.cc
  camera_sensor.cc
  gpu_laser.cc
  gz_camera.cc
  heightmap.cc
  multicamera_sensor.cc
  ogre_log.cc
  pr2.cc
  projector.cc
  rendering_sensor.cc
  speed_pr2.cc
  world_reset.cc
)

gz_build_dri_tests(${dri_tests})


if (ENABLE_TESTS_COMPILATION AND VALID_DRI_DISPLAY)
  # Increase timeout, to account for model download time.
  set_tests_properties(${TEST_TYPE}_pr2 PROPERTIES TIMEOUT 500)

  # Increase timeout, to account for model download time.
  set_tests_properties(${TEST_TYPE}_pioneer2dx PROPERTIES TIMEOUT 500)
endif()

