cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(test_fixture_example)

find_package(gazebo REQUIRED)
include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED filesystem)

include_directories(${GAZEBO_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/gtest/include
  ${PROJECT_SOURCE_DIR}/gtest
  ${PROJECT_SOURCE_DIR}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)
set(GTEST_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest.a")
set(GTEST_MAIN_LIBRARY "${PROJECT_BINARY_DIR}/test/libgtest_main.a")
include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test test.cc)
target_link_libraries(test libgtest.a libgtest_main.a gazebo_test_fixture
  ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
