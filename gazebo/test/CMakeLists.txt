# Build ServerFixture library
add_library(gazebo_test_fixture STATIC ServerFixture.cc ServerFixtureRecord.cc)
target_link_libraries(gazebo_test_fixture libgazebo)

gz_install_library(gazebo_test_fixture)
gz_install_includes("test"
  helper_physics_generator.hh
  ServerFixture.hh
  ${PROJECT_BINARY_DIR}/test_config.h
)
