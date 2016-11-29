cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# Find the Ignition_Transport library
find_package(ignition-transport2 QUIET)
if (NOT ignition-transport2_FOUND)
  find_package(ignition-transport1 REQUIRED)
endif()
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})

add_executable(client client.cc)
target_link_libraries(client ${GAZEBO_LIBRARIES} ${IGNITION-TRANSPORT_LIBRARIES})
