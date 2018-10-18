cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include (FindPkgConfig)

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(arrange_plugin_test arrange_plugin_test.cc)
target_link_libraries(arrange_plugin_test ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES})
