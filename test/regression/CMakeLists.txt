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

set(tests
  bandwidth.cc
  contact_sensor.cc
  factory.cc
  file_handling.cc
  imu.cc
  laser.cc
  nondefault_world.cc
  physics.cc
  pioneer2dx.cc
  transport.cc
  server_fixture.cc
  speed.cc
  #state_log.cc
  surface_properties.cc
  )

set (GZ_BUILD_TESTS_EXTRA_EXE_SRCS
  "${PROJECT_SOURCE_DIR}/gazebo/Server.cc;${PROJECT_SOURCE_DIR}/gazebo/Master.cc;${PROJECT_SOURCE_DIR}/gazebo/gazebo.cc;${PROJECT_SOURCE_DIR}/test/ServerFixture.cc")
gz_build_tests(${tests})

set(display_tests
)

# Build the display tests (need extra sources to compile)
set (GZ_BUILD_TESTS_EXTRA_EXE_SRCS
    "${PROJECT_SOURCE_DIR}/gazebo/Server.cc;${PROJECT_SOURCE_DIR}/gazebo/Master.cc;${PROJECT_SOURCE_DIR}/gazebo/gazebo.cc;${PROJECT_SOURCE_DIR}/test/ServerFixture.cc")
gz_build_display_tests(${display_tests})

set(dri_tests
  camera_sensor.cc
  gpu_laser.cc
  heightmap.cc
  pr2.cc
  projector.cc
  speed_pr2.cc
)

set (GZ_BUILD_TESTS_EXTRA_EXE_SRCS
    "${PROJECT_SOURCE_DIR}/gazebo/Server.cc;${PROJECT_SOURCE_DIR}/gazebo/Master.cc;${PROJECT_SOURCE_DIR}/gazebo/gazebo.cc;${PROJECT_SOURCE_DIR}/test/ServerFixture.cc")
gz_build_dri_tests(${dri_tests})


if (VALID_DRI_DISPLAY)
  # Increase timeout, to account for model download time.
  set_tests_properties(${TEST_TYPE}_pr2 PROPERTIES TIMEOUT 500)

  # Increase timeout, to account for model download time.
  set_tests_properties(${TEST_TYPE}_pioneer2dx PROPERTIES TIMEOUT 500)
endif()
