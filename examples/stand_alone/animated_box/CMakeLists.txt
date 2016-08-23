cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(animated_box)

# Find packages

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create libraries and executables

add_library(animated_box SHARED animated_box.cc)
target_link_libraries(animated_box ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(integrated_main integrated_main.cc)
target_link_libraries(integrated_main ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)


add_executable(independent_listener independent_listener.cc)
target_link_libraries(independent_listener ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
