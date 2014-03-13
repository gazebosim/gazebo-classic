set (tests example_plugins.cc)
add_definitions("-DCMAKE_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"")
gz_build_tests(${tests})
