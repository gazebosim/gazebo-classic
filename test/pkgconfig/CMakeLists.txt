set (tests config-pkgconfig.cc)
add_definitions(-DSOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/plugin\" -DGAZEBO_EXPECTED_VERSION="${GAZEBO_VERSION_FULL}")
gz_build_tests(${tests})
