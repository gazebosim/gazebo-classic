include ( ${QT_USE_FILE} )
add_definitions(${QT_DEFINITIONS})

include_directories(
  ${tinyxml_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDF_INCLUDE_DIRS}
)

link_directories(
  ${SDF_LIBRARY_DIRS}
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

add_executable(gzsdf gzsdf.cc)
target_link_libraries(gzsdf ${tinyxml_LIBRARIES} gazebo_common)
gz_install_executable(gzsdf)

add_executable(gztopic gztopic.cc)
target_link_libraries(gztopic gazebo_msgs gazebo_common gazebo_transport pthread gazebo_gui ${QT_LIBRARIES})
gz_install_executable(gztopic)

add_executable(gzlog gzlog.cc)
target_link_libraries(gzlog gazebo_common gazebo_physics gazebo_transport gazebo_sensors gazebo_msgs pthread)
gz_install_executable(gzlog)

add_executable(gzstats gzstats.cc)
target_link_libraries(gzstats gazebo_msgs gazebo_common gazebo_transport pthread)
gz_install_executable(gzstats)

add_executable(gzfactory gzfactory.cc)
target_link_libraries(gzfactory gazebo_common gazebo_transport pthread)
gz_install_executable(gzfactory)

if (${GAZEBO_VERSION} VERSION_LESS 4.0)
  message (WARNING
    "Installing deprecated manpages. This should be removed after Gazebo 3.0")
  manpage(gzfactory 1)
  manpage(gzlog 1)
  manpage(gzsdf 1)
  manpage(gzstats 1)
  manpage(gztopic 1)
endif()

add_executable(gz gz.cc gz_topic.cc gz_log.cc)
target_link_libraries(gz
 gazebo_msgs gazebo_common gazebo_transport gazebo_gui gazebo_physics
 gazebo_sensors ${QT_LIBRARIES} ${Boost_LIBRARIES} pthread)
gz_install_executable(gz)

roffman(gz 1)

install (PROGRAMS gzprop DESTINATION ${BIN_INSTALL_DIR})
manpage(gzprop 1)
