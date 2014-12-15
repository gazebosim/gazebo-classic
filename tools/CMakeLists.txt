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
  gzlog_TEST.cc
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

install (PROGRAMS gzmodel_create DESTINATION ${BIN_INSTALL_DIR})

install (PROGRAMS gzprop DESTINATION ${BIN_INSTALL_DIR})
manpage(gzprop 1)
