cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
if (NOT "${GAZEBO_VERSION}" STREQUAL "${GAZEBO_EXPECTED_VERSION}")
  message(FATAL_ERROR "Expected version is '${GAZEBO_EXPECTED_VERSION}' but found '${GAZEBO_VERSION}' in '${GAZEBO_PREFIX}'")
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(hello_world SHARED ../../testfiles/hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
