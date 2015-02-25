include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories (
  ${PROJECT_BINARY_DIR}/gazebo
  ${PROTOBUF_INCLUDE_DIR}
)

link_directories(${PROJECT_BINARY_DIR}/test)

set (gtest_sources 
  gz_log.cc 
)

gz_build_tests(${gtest_sources})
