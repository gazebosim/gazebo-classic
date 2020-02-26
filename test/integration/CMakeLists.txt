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
set (CMAKE_AUTOMOC ON)

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
  attach_light_plugin.cc
  bandwidth.cc
  concave_mesh.cc
  contact_sensor.cc
  contacts_update.cc
  contain_plugin.cc
  dem.cc
  elastic_modulus.cc
  file_handling.cc
  flash_light_plugin.cc
  gripper.cc
  gz_joint.cc
  gz_log.cc
  gz_model.cc
  gz_physics.cc
  gz_world.cc
  harness.cc
  imu.cc
  info_services.cc
  introspection_items.cc
  joint_control_plugin.cc
  joint_controller.cc
  joint_force_torque.cc
  joint_gearbox.cc
  joint_get_force_torque.cc
  joint_revolute.cc
  joint_screw.cc
  joint_set_position_test.cc
  joint_spawn.cc
  joint_test.cc
  joint_universal.cc
  keys_to_joints.cc
  kinematic_loop.cc
  laser.cc
  led_plugin.cc
  link.cc
  logical_camera_sensor.cc
  misalignment_plugin.cc
  model.cc
  model_database.cc
  multirayshape.cc
  nested_model.cc
  noise.cc
  nondefault_world.cc
  obj_loader.cc
  physics.cc
  physics_base.cc
  physics_basic_controller_response.cc
  physics_collision.cc
  physics_friction.cc
  physics_inertia_ratio.cc
  physics_link.cc
  physics_msgs.cc
  physics_msgs_inertia.cc
  physics_presets.cc
  physics_solver.cc
  physics_thread_safe.cc
  physics_torsional_friction.cc
  pioneer2dx.cc
  plugin.cc
  plugin_interface.cc
  rayshape.cc
  region_events.cc
  rest_web.cc
  saving_and_loading.cc
  sdf.cc
  sensor.cc
  server_fixture.cc
  sim_events.cc
  speed.cc
  speed_thread_islands.cc
  speed_thread_pr2.cc
  static_map_plugin.cc
  stress_spawn_models.cc
  #state_log.cc
  surface_properties.cc
  swarm.cc
  touch_plugin.cc
  tracked_vehicles.cc
  transceiver.cc
  transport.cc
  transporter.cc
  variable_gearbox_plugin.cc
  wheel_slip.cc
  world.cc
  world_clone.cc
  world_entity_below_point.cc
  world_playback.cc
  world_population.cc
  worlds_installed.cc
  )

if (HAVE_GTS)
  set(tests
    ${tests}
    polyline.cc
  )
endif()

if (MANPAGES_SUPPORT)
  set (tests ${tests}
	      manpages.cc)
endif()

gz_build_tests(${tests} EXTRA_LIBS gazebo_test_fixture)

# Increase timeout, to account for model download time.
set_tests_properties(${TEST_TYPE}_joint_revolute PROPERTIES TIMEOUT 500)
set_tests_properties(${TEST_TYPE}_model_database PROPERTIES TIMEOUT 400)
set_tests_properties(${TEST_TYPE}_physics PROPERTIES TIMEOUT 500)
set_tests_properties(${TEST_TYPE}_worlds_installed PROPERTIES TIMEOUT 700)

set_property(
  SOURCE world_clone.cc
  PROPERTY COMPILE_DEFINITIONS
  GAZEBO_DEFAULT_MASTER_PORT=${GAZEBO_DEFAULT_MASTER_PORT}
)

# Increase timeout for debug-mode
if (${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "DEBUG" OR
    ${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "COVERAGE")
  set_tests_properties(${TEST_TYPE}_joint_gearbox PROPERTIES TIMEOUT 400)
endif()

# Add plugin dependency
add_dependencies(${TEST_TYPE}_joint_control_plugin JointControlPlugin)
add_dependencies(${TEST_TYPE}_joint_test SpringTestPlugin)
add_dependencies(${TEST_TYPE}_plugin_interface PluginInterfaceTest)
add_dependencies(${TEST_TYPE}_tracked_vehicles SimpleTrackedVehiclePlugin)
add_dependencies(${TEST_TYPE}_tracked_vehicles WheelTrackedVehiclePlugin)
add_dependencies(${TEST_TYPE}_variable_gearbox_plugin VariableGearboxPlugin)

set(display_tests
)

# Build the display tests (need extra sources to compile)
gz_build_display_tests(${display_tests})

set(dri_tests
  camera.cc
  camera_sensor.cc
  camera_sensor_igntransport.cc
  depth_camera.cc
  factory.cc
  fiducial.cc
  gpu_laser.cc
  gz_camera.cc
  heightmap.cc
  multicamera_sensor.cc
  ogre_log.cc
  pr2.cc
  projector.cc
  rendering_sensor.cc
  road.cc
  speed_pr2.cc
  visual.cc
  wideanglecamera_sensor.cc
  world_remove.cc
  world_reset.cc
)
gz_build_dri_tests(${dri_tests} EXTRA_LIBS gazebo_test_fixture)

set(qt_tests
  camera_gui.cc
  collada_visualization.cc
  force_torque_visualization.cc
  insert_model.cc
  joint_control_widget.cc
  marker.cc
  model_editor.cc
  model_editor_undo.cc
  model_manipulation.cc
  mouse_pick.cc
  track_visual.cc
  undo.cc
  view_angle.cc
  view_control.cc
  visual_pose.cc
)

gz_build_qt_tests(${qt_tests})

if (VALID_DRI_DISPLAY)
  add_dependencies(${TEST_TYPE}_heightmap HeightmapLODPlugin)
  # Increase timeout, to account for model download time.
  set_tests_properties(${TEST_TYPE}_factory PROPERTIES TIMEOUT 500)
  set_tests_properties(${TEST_TYPE}_pr2 PROPERTIES TIMEOUT 500)
  set_tests_properties(${TEST_TYPE}_pioneer2dx PROPERTIES TIMEOUT 500)
endif()
