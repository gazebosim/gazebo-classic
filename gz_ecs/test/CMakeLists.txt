
# Copying gtest here because gazebo's existing tests take too long
# for rapid prototyping

include_directories(gtest/)
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)
include_directories(${GTEST_INCLUDE_DIRS})

add_subdirectory(gtest)

include_directories(include)
add_subdirectory(util)

add_subdirectory(unit_tests)
