cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(projector_plugin SHARED projector.cc)
target_link_libraries(projector_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
