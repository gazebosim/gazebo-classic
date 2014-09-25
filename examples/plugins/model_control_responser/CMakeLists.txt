cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# find simbody
find_package(Simbody REQUIRED)
include_directories(SYSTEM ${Simbody_INCLUDE_DIR})
link_directories(${Simbody_LIB_DIR})
# add_definitions(${Simbody_CFLAGS})

add_executable(responser responser.cc UR10.h UR10.cpp)
target_link_libraries(responser ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread ${Simbody_LIBRARIES})

