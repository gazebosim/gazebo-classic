cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} )

add_library(system_gui SHARED system_gui.cc)
target_link_libraries(system_gui ${GAZEBO_LIBRARIES})
