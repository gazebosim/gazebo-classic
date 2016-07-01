cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(ignition-math2)
find_package(Gazebo)

set(CMAKE_CXX_FLAGS ${IGNITION-MATH_CXX_FLAGS})
include_directories(${GAZEBO_INCLUDE_DIRS} ${IGNITION-MATH_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${IGNITION-MATH_LIBRARY_DIRS})

add_executable(vector3 vector3.cc)
target_link_libraries(vector3 ${GAZEBO_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

add_executable(box box.cc)
target_link_libraries(box ${GAZEBO_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

add_executable(angle angle.cc)
target_link_libraries(angle ${GAZEBO_LIBRARIES} ${IGNITION-MATH_LIBRARIES})
