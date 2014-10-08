cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

# Find the Ignition_Transport library
find_package(ignition-transport QUIET REQUIRED)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")

# find DART
find_package(DARTCore 4.1 REQUIRED)
include_directories(SYSTEM
  ${IGNITION-TRANSPORT_INCLUDE_DIRS}
  ${DARTCore_INCLUDE_DIR}
  ${GAZEBO_INCLUDE_DIRS})
link_directories(
  ${IGNITION-TRANSPORT_LIBRARY_DIRS}
  ${DARTCore_LIB_DIR}
  ${GAZEBO_LIBRARY_DIRS})

add_executable(responser responser.cc)
target_link_libraries(responser
  ${IGNITION-TRANSPORT_LIBRARIES}
  gazebo_msgs #${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  pthread
  ${DARTCore_LIBRARIES})

