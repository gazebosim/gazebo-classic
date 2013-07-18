include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories (
  ${PROTOBUF_INCLUDE_DIR}
)

set (gtest_sources 
  MsgFactory.cc 
)

gz_build_tests(${gtest_sources})
