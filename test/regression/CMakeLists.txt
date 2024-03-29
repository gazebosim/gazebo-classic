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

if(HAVE_BULLET)
  include_directories ( ${BULLET_INCLUDE_DIRS} )
  link_directories ( ${BULLET_LIBRARY_DIRS} )
endif()
if(HAVE_DART)
  include_directories ( ${DART_INCLUDE_DIRS} )
  link_directories ( ${DART_LIBRARY_DIRS} )
endif()

set(tests
  351_world_step.cc
  494_joint_axis_frame.cc
  602_unsubscribe_segfault.cc
  624_collision_world_pose.cc
  876_random_number_generator.cc
  940_force_torque_sensor_frame.cc
  978_joint_anchor.cc
  1082_pid_limits.cc
  1112_force_torque_model_removal.cc
  1124_collision_pose.cc
  1146_collision_bounding_box.cc
  1208_world_plugin_init.cc
  1375_world_reset.cc
  1569_hydra_crash.cc
  1694_world_accel.cc
  1782_unadvertise.cc
  2239_revolute2_teleport.cc
  2297_log_insertions_paused.cc
  2428_log_insertions.cc
  2430_revolute_joint_SetPosition.cc
  2505_revolute_joint_SetAxis.cc
  2527_distortion_leak.cc
  2728_nested_urdf.cc
  2875_connect_pub_to_sub_crash.cc
  2896_gazebo_subnamespace.cc
  2902_performance_metrics_deadlock.cc
  3125_slow_loading_model_spawn.cc
)
gz_build_tests(${tests} EXTRA_LIBS gazebo_test_fixture)

add_dependencies(${TEST_TYPE}_3125_slow_loading_model_spawn
                 SlowLoadingSensorPlugin
                 WorldSpawnModelPlugin)

set(display_tests
)

# Build the display tests (need extra sources to compile)
gz_build_display_tests(${display_tests})

set(dri_tests
  346_save_lights.cc
  846_typo_in_camera.cc
  1702_remove_model_scene_nodes.cc
)
gz_build_dri_tests(${dri_tests} EXTRA_LIBS gazebo_test_fixture)

add_dependencies(${TEST_TYPE}_1112_force_torque_model_removal
                 ForceTorqueModelRemovalTestPlugin)
add_dependencies(${TEST_TYPE}_1208_world_plugin_init
                 Issue1208Plugin)

set (CMAKE_AUTOMOC ON)
set(qt_tests
  2728_gui_spawned_nested_urdf.cc
)

gz_build_qt_tests(${qt_tests})
