# These are Unit tests. They run very quickly and test as little of the
#   codebase that they can get away with. It should be possible to run
#   all unit tests in just a few seconds. If a test takes longer than
#   a milisecond to run, there's a good chance it's a different kind of
#   test
set(unit_tests
  EntityComponentDatabase_TEST.cc
  EntityQuery_TEST.cc
  # SystemManager_TEST.cc
)


# Loop to take care of linking and test macros
# This makes targets like UNIT_EntityManager_TEST
foreach (src_file ${unit_tests})
  string(REGEX REPLACE "\\.cc" "" BINARY_NAME ${src_file})
  set(BINARY_NAME UNIT_${BINARY_NAME})
  add_executable(${BINARY_NAME} ${src_file})
  target_link_libraries(${BINARY_NAME} gtest gtest_main GazeboECS ${IGNITION-COMMON_LIBRARIES})
  if (UNIX)
    # gtest uses pthread on UNIX
    target_link_libraries(${BINARY_NAME} pthread)
  endif()
  add_test(${BINARY_NAME} ${CMAKE_CURRENT_BINARY_DIR}/${BINARY_NAME}
    --gtest_output=xml:${CMAKE_BINARY_DIR}/test_results/${BINARY_NAME}.xml)
endforeach()
