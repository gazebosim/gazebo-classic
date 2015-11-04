cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(test_fixture_example)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/gtest/include
  ${PROJECT_SOURCE_DIR}/gtest
)

link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)

add_executable(test_example test.cc)
target_link_libraries(test_example gtest gtest_main gazebo_test_fixture
  ${GAZEBO_LIBRARIES})
