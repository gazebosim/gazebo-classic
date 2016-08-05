cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(ignition-transport 1 REQUIRED)
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

find_package(ignition-msgs 0 REQUIRED)
include_directories(${IGNITION-MSGS_INCLUDE_DIRS})
link_directories(${IGNITION-MSGS_LIBRARY_DIRS})

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set (CMAKE_CXX_FLAGS
  "${CMAKE_CXX_FLAGS} ${IGNITION-MSGS_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")

add_executable(marker marker.cc)
target_link_libraries(marker ${GAZEBO_LIBRARIES} ${IGNITION-MSGS_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES} pthread)
