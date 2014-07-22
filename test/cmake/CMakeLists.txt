set (tests config-cmake.cc)
add_definitions(-DSOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/plugin\" -DGAZEBO_VERSION=${GAZEBO_VERSION})
gz_build_tests(${tests})
