cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  string (REPLACE ";" " " GAZEBO_CFLAGS "${GAZEBO_CFLAGS}")
endif()
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}/msgs
  )
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CFLAGS}")

add_subdirectory(msgs)

add_library(custom_messages SHARED custom_messages.cc)
target_link_libraries(custom_messages my_msgs ${GAZEBO_libraries})
