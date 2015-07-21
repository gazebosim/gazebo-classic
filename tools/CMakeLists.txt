include ( ${QT_USE_FILE} )
add_definitions(${QT_DEFINITIONS})

include_directories(
  ${tinyxml_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories(
  ${CCD_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  ${tinyxml_LIBRARY_DIRS}
)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

if (HAVE_DART)
  link_directories(${DARTCore_LIBRARY_DIRS})
endif()

if (CURL_FOUND)
  include_directories(${CURL_INCLUDEDIR})
  link_directories(${CURL_LIBDIR})
  if (WIN32)
    add_definitions(-DCURL_STATICLIB)
  endif()
endif()


set (test_sources
  gz_log_TEST.cc
  gz_TEST.cc
)

gz_build_tests(${test_sources})

add_executable(gz gz.cc gz_topic.cc gz_log.cc)
target_link_libraries(gz
 libgazebo_client
 gazebo_msgs gazebo_common gazebo_transport gazebo_gui gazebo_physics
 gazebo_physics_ode gazebo_sensors ${QT_LIBRARIES} ${Boost_LIBRARIES})

if(HAVE_BULLET)
  target_link_libraries(gz gazebo_physics_bullet)
endif()
if (HAVE_SIMBODY)
  target_link_libraries(gz gazebo_physics_simbody)
endif()
if(HAVE_DART)
  target_link_libraries(gz gazebo_physics_dart)
endif()

if (UNIX)
  target_link_libraries(gz pthread)
endif()

gz_install_executable(gz)

if (NOT WIN32)
  roffman(gz 1)
endif()

install (PROGRAMS gzprop DESTINATION ${BIN_INSTALL_DIR})

if (NOT WIN32)
  manpage(gzprop 1)
endif()
