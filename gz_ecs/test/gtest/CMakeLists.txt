# Copying gtest here because gazebo's existing tests take a long time
include_directories(include .)
add_library(gtest STATIC src/gtest-all.cc)
add_library(gtest_main STATIC src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
