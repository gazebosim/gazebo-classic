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
)

set (GZ_BUILD_TESTS_EXTRA_EXE_SRCS
    "${PROJECT_SOURCE_DIR}/gazebo/Server.cc;${PROJECT_SOURCE_DIR}/gazebo/Master.cc;${PROJECT_SOURCE_DIR}/gazebo/gazebo.cc;${PROJECT_SOURCE_DIR}/test/ServerFixture.cc")
gz_build_dri_tests(${dri_tests})
