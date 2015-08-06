cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mode_move)

# Find packages

find_package(Gazebo REQUIRED)

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Create libraries and executables

add_executable(path_publisher path_publisher.cc)
target_link_libraries(path_publisher ${GAZEBO_LIBRARIES})

add_library(model_move SHARED model_move.cc)
target_link_libraries(model_move ${GAZEBO_LIBRARIES})
