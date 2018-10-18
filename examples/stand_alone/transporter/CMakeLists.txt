cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(transporter transporter.cc)
target_link_libraries(transporter ${GAZEBO_LIBRARIES})

if (UNIX)
  target_link_libraries(transporter pthread)
endif()
