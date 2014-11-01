cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
include (FindPkgConfig)

if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(arrange_plugin_test arrange_plugin_test.cc)
target_link_libraries(arrange_plugin_test ${GAZEBO_LIBRARIES} ${PROTOBUF_LIBRARIES})
