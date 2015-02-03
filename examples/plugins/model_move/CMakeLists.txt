cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mode_move)

# Find packages

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(Protobuf REQUIRED)
find_package(sdformat REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()


# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create libraries and executables

add_library(model_move SHARED model_move.cc)
target_link_libraries(model_move ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(model_move_path SHARED model_move_path.cc)
target_link_libraries(model_move_path ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_executable(integrated_main integrated_main.cc)
target_link_libraries(integrated_main ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)

add_executable(independent_listener independent_listener.cc)
target_link_libraries(independent_listener ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
