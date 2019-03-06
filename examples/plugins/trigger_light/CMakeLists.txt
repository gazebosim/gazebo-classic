cmake_minimum_required(VERSION 2.8.12)
project(example_trigger_light)

find_package(gazebo REQUIRED)
find_package(ignition-transport4 REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(TurnOnLightPlugin SHARED TurnOnLightPlugin.cpp)
target_link_libraries(TurnOnLightPlugin
  ${GAZEBO_LIBRARIES}
  ignition-transport4::ignition-transport4
)
