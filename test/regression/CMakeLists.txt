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
  include_directories ( ${DARTCore_INCLUDE_DIRS} )
  link_directories ( ${DARTCore_LIBRARY_DIRS} )
endif()

set(tests
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
)

gz_build_tests(${tests})

set(display_tests
)

# Build the display tests (need extra sources to compile)
gz_build_display_tests(${display_tests})

set(dri_tests
  346_save_lights.cc
  846_typo_in_camera.cc
)

gz_build_dri_tests(${dri_tests})

if (ENABLE_TESTS_COMPILATION)
  add_dependencies(${TEST_TYPE}_1112_force_torque_model_removal
                   ForceTorqueModelRemovalTestPlugin)
  add_dependencies(${TEST_TYPE}_1208_world_plugin_init Issue1208Plugin)
endif()
