cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(animate_pose SHARED animate_pose.cc)
target_link_libraries(animate_pose ${GAZEBO_LIBRARIES})
