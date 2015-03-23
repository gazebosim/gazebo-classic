cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(IGNITION-MATH ignition-math)

  # todo: Put this line in when ignition-math's pkg-config is updated to have
  # -std=c++11. There is another related todo below
  # string (REPLACE ";" " " IGNITION-MATH_CFLAGS "${IGNITION-MATH_CFLAGS}")
endif()
include_directories(${GAZEBO_INCLUDE_DIRS} ${IGNITION-MATH_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS} ${IGNITION-MATH_LIBRARY_DIRS})

# todo: replace this line with the commented out one below
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
# set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-MATH_CFLAGS}")

add_library(hello_world SHARED ../../testfiles/hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES} ${IGNITION-MATH_LIBRARIES} ${Boost_LIBRARIES})
