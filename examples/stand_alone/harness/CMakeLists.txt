cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(harness_detach harness_detach.cc)
target_link_libraries(harness_detach ${GAZEBO_LIBRARIES})

add_executable(harness_velocity harness_velocity.cc)
target_link_libraries(harness_velocity ${GAZEBO_LIBRARIES})
