cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# include(FindBoost)
# find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(ignition-transport0 QUIET)
if (NOT ignition-transport0_FOUND)
  message(FATAL "Missing: Ignition Transport (libignition-transport0-dev)")
else()
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}")
  include_directories(${IGNITION-TRANSPORT_INCLUDE_DIRS})
  link_directories(${IGNITION-TRANSPORT_LIBRARY_DIRS})
endif()

find_package(gazebo REQUIRED)
find_package(Protobuf REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_executable(marker marker.cc)
target_link_libraries(marker ${GAZEBO_LIBRARIES} ${IGNTION-TRANSPORT_LIBRARIES}
  ${PROTOBUF_LIBRARIES} pthread)
