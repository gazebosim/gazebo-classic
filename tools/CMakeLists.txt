include ( ${QT_USE_FILE} )
add_definitions(${QT_DEFINITIONS})

include_directories( 
  ${tinyxml_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

if (WIN32)
  include_directories(${libdl_include_dir} ${TBB_INCLUDEDIR})
endif()

link_directories( 
  ${SDFormat_LIBRARY_DIRS}
  ${tinyxml_LIBRARY_DIRS}
  ${CURL_LIBDIR}
  ${Boost_LIBRARY_DIR}
  ${TBB_LIBRARY_DIR}
)

add_definitions(-DCURL_STATICLIB)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

set (test_sources 
  gz_log_TEST.cc 
  gz_TEST.cc
)

gz_build_tests(${test_sources})

# TODO: port gzlog to windows
if (WIN32)
  add_executable(gz gz.cc gz_topic.cc gz_log_stub.cc)
else()
  add_executable(gz gz.cc gz_topic.cc gz_log.cc)
endif()

target_link_libraries(gz 
	gazebo_msgs 
	gazebo_common 
	gazebo_util
	gazebo_transport 
	gazebo_gui 
	gazebo_physics
	gazebo_physics_ode
	gazebo_sensors 
	${QT_LIBRARIES} 
	${Boost_LIBRARIES})

if (UNIX)
  target_link_libraries(gz pthread)
endif()

gz_install_executable(gz)

roffman(gz 1)

install (PROGRAMS gzprop DESTINATION ${BIN_INSTALL_DIR})
manpage(gzprop 1)
