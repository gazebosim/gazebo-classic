set (tests config-cmake.cc)
# Using GAZEBO_VERSION and not GAZEBO_VERSION_FULL since
# probably this way is closer to users implementation, using just
# two version number.
add_definitions(-DSOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/plugin\" -DGAZEBO_VERSION=${GAZEBO_VERSION})
gz_build_tests(${tests})
