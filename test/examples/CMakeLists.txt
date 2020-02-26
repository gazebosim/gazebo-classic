set (tests
  examples_build.cc
)
gz_build_tests(${tests} EXTRA_LIBS ${Boost_LIBRARIES})

# Increase timeout since there are many tests
set_tests_properties(${TEST_TYPE}_examples_build PROPERTIES TIMEOUT 500)
