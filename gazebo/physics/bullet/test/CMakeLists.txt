
include_directories (
  ${PROJECT_SOURCE_DIR}/gazebo 
  ${PROJECT_BINARY_DIR}/gazebo
  ${PROTOBUF_INCLUDE_DIR}
  ${BULLET_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_BINARY_DIR}/test
  ${BULLET_LIBRARY_DIRS}
)

set (gtest_sources 
  BulletTypes.cc 
)

# Build tests
foreach(GTEST_SOURCE_file ${gtest_sources})
  string(REGEX REPLACE ".cc" "" BINARY_NAME ${GTEST_SOURCE_file})
  add_executable(${BINARY_NAME} ${GTEST_SOURCE_file})
  add_dependencies(${BINARY_NAME} gtest)

  target_link_libraries(${BINARY_NAME}
    libgtest.a
    libgtest_main.a 
    gazebo_common
    gazebo_msgs
    pthread
    )

  add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME} --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
  set_tests_properties(${BINARY_NAME} PROPERTIES TIMEOUT 240)

  # Check that the test produced a result and create a failure if it didn't.
  # Guards against crashed and timed out tests.
  add_test(check_${BINARY_NAME} ${PROJECT_SOURCE_DIR}/tools/check_test_ran.py ${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
endforeach()
