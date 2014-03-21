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
  gripper.cc
  )

if (MANPAGES_SUPPORT)
  set (tests ${tests}
	      manpages.cc)
endif()

gz_build_tests(${tests})

set(display_tests
)

# Build the display tests (need extra sources to compile)
gz_build_display_tests(${display_tests})

set(dri_tests
  camera_sensor.cc
  multicamera_sensor.cc
  gpu_laser.cc
  gz_camera.cc
  camera_follow.cc
  heightmap.cc
  ogre_log.cc
  pr2.cc
  projector.cc
  rendering_sensor.cc
  speed_pr2.cc
  world_reset.cc
)
