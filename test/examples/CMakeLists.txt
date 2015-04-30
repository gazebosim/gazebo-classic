set (tests
  examples_build.cc
)
add_definitions("-DCMAKE_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"")
gz_build_tests(${tests})
