add_definitions(${QT_DEFINITIONS})

include_directories(
  ${tinyxml_INCLUDE_DIRS}
  ${tinyxml2_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
)

link_directories(
  ${CCD_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  ${tinyxml_LIBRARY_DIRS}
  ${IGNITION-MATH_LIBRARY_DIRS}
)
set (CMAKE_AUTOMOC ON)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

if (HAVE_DART)
  link_directories(${DART_LIBRARY_DIRS})
endif()

if ("${GZ_CLI_EXECUTABLE_NAME}" STREQUAL "")
  set(GZ_CLI_EXECUTABLE_NAME "gz")
endif()

if(NOT WIN32)
  # gz_TEST and gz_log_TEST use fork(), that is not available on Windows
  set (test_sources
    gz_log_TEST.cc
    gz_TEST.cc
  )

  gz_build_tests(${test_sources} EXTRA_LIBS libgazebo_client gazebo_transport gazebo_util)
  add_dependencies(${TEST_TYPE}_gz_log_TEST ${GZ_CLI_EXECUTABLE_NAME})
endif()

add_executable(${GZ_CLI_EXECUTABLE_NAME} gz.cc gz_topic.cc gz_log.cc gz_marker.cc)

if (WIN32)
  # Force multiple definitions since there is a collision with sdformat GetAsEuler() function
  # https://github.com/osrf/sdformat/blob/master/include/sdf/Types.hh
  # it is defined inside an .hh file and bring to gz linking via sdformat and gazebo_gui/gazebo_common
  set_target_properties(${GZ_CLI_EXECUTABLE_NAME} PROPERTIES LINK_FLAGS "/FORCE:MULTIPLE")
endif()

target_link_libraries(${GZ_CLI_EXECUTABLE_NAME}
 libgazebo_client
 gazebo_gui
 gazebo_physics
 gazebo_sensors
 gazebo_transport
 ${Qt5Core_LIBRARIES}
 ${Qt5Widgets_LIBRARIES}
 ${Boost_LIBRARIES}
 ${IGNITION-TRANSPORT_LIBRARIES}
)

if (UNIX)
  target_link_libraries(${GZ_CLI_EXECUTABLE_NAME} pthread)
endif()

gz_install_executable(${GZ_CLI_EXECUTABLE_NAME})

if (NOT WIN32)
  roffman(gz 1 ${GZ_CLI_EXECUTABLE_NAME})
endif()

install (PROGRAMS gzprop DESTINATION ${BIN_INSTALL_DIR})

if (NOT WIN32)
  manpage(gzprop 1)
endif()
