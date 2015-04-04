cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
string (REPLACE ";" " " GAZEBO_CFLAGS "${GAZEBO_CFLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CFLAGS}")

add_library(factory SHARED factory.cc)
target_link_libraries(factory
  ${GAZEBO_LIBRARIES}
)
