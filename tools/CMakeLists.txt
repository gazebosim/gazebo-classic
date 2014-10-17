include ( ${QT_USE_FILE} )
add_definitions(${QT_DEFINITIONS})

include_directories( 
  ${tinyxml_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
)

link_directories( 
  ${SDFormat_LIBRARY_DIRS}
  ${tinyxml_LIBRARY_DIRS}
)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

set (test_sources 
  gz_log_TEST.cc 
  gz_TEST.cc
)

gz_build_tests(${test_sources})

add_executable(gz gz.cc gz_topic.cc gz_log.cc)
target_link_libraries(gz
 gazebo_msgs gazebo_common gazebo_transport gazebo_gui gazebo_physics
 gazebo_sensors ${QT_LIBRARIES} ${Boost_LIBRARIES} pthread)
gz_install_executable(gz)

roffman(gz 1)

install (PROGRAMS gzprop DESTINATION ${BIN_INSTALL_DIR})
manpage(gzprop 1)
