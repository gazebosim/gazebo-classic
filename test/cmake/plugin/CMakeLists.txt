cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# This GAZEBO_VERSION does not come from gazebo cmake system, it is passed
# to this test using add_definitions in the CMakeLists.txt in ../ directory 
find_package(gazebo ${GAZEBO_VERSION} REQUIRED)
if (NOT gazebo_FOUND)
  message(FATAL_ERROR "Didn't find Gazebo")
endif()
if (NOT GAZEBO_LIBRARIES)
  message(FATAL_ERROR "Empty GAZEBO_LIBRARIES")
endif()
if (NOT GAZEBO_INCLUDE_DIRS)
  message(FATAL_ERROR "Empty GAZEBO_LIBRARIES")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(hello_world SHARED ../../testfiles/hello_world.cc)
target_link_libraries(hello_world ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
