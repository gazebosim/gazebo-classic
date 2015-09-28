set (tests
  examples_build.cc
)
add_definitions("-DCMAKE_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"")
gz_build_tests(${tests})

if (ENABLE_TESTS_COMPILATION)
  # Increase timeout since there are many tests
  set_tests_properties(${TEST_TYPE}_examples_build PROPERTIES TIMEOUT 500)
endif()
