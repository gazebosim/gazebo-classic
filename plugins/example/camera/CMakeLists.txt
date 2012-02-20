cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(camera_dump SHARED camera_dump.cc)
target_link_libraries(camera_dump ${GAZEBO_libraries} CameraPlugin)

add_library(camera_move SHARED camera_move.cc)
target_link_libraries(camera_move ${GAZEBO_libraries})
