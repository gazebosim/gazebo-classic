cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include(FindBoost)

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)
find_package(SDFormat REQUIRED)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex program_options)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

include_directories(
  ${SDFormat_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(
  ${SDFormat_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)

add_executable(actuator main.cc)
target_link_libraries(actuator
  ${SDFormat_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
)
