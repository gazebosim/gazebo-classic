cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(pub_new_world pub_new_world.cc)
add_executable(pub_open_world pub_open_world.cc)

target_link_libraries(pub_new_world ${GAZEBO_LIBRARIES})
target_link_libraries(pub_open_world ${GAZEBO_LIBRARIES})
