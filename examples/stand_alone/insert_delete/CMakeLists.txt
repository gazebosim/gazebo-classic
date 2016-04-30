cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(delete_ball delete_ball.cc)
add_executable(insert_ball insert_ball.cc)
target_link_libraries(delete_ball ${GAZEBO_LIBRARIES})
target_link_libraries(insert_ball ${GAZEBO_LIBRARIES})
