cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem regex)

find_package(Protobuf REQUIRED)

find_package(SDFormat REQUIRED)

include_directories(${SDFormat_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
link_directories(${SDFormat_LIBRARY_DIRS} ${GAZEBO_LIBRARY_DIRS})

add_executable(actuator main.cc)
target_link_libraries(actuator ${SDFormat_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${PROTOBUF_LIBRARIES} pthread)
