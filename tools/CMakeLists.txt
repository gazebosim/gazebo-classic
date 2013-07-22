link_directories(${urdfdom_LIBRARY_DIRS} ${urdfdom_headers_LIBRARY_DIRS} ${console_bridge_LIBRARY_DIRS})

include ( ${QT_USE_FILE} )
add_definitions(${QT_DEFINITIONS})


include_directories( 
  ${tinyxml_include_dirs}
  ${PROTOBUF_INCLUDE_DIR}
)

set (test_sources 
  gzlog_TEST.cc 
)

gz_build_tests(${test_sources})

add_executable(gzsdf gzsdf.cc)
target_link_libraries(gzsdf ${tinyxml_libraries} gazebo_sdf_interface)
gz_install_executable(gzsdf)

add_executable(gztopic gztopic.cc)
target_link_libraries(gztopic gazebo_msgs gazebo_common gazebo_transport pthread gazebo_gui ${QT_LIBRARIES})
gz_install_executable(gztopic)

add_executable(gzlog gzlog.cc)
target_link_libraries(gzlog gazebo_common gazebo_physics gazebo_transport gazebo_sensors gazebo_msgs gazebo_sdf_interface pthread)
gz_install_executable(gzlog)

add_executable(gzstats gzstats.cc)
target_link_libraries(gzstats gazebo_msgs gazebo_common gazebo_transport pthread)
gz_install_executable(gzstats)

#add_executable(sdf2pov sdf2pov.cc)
#target_link_libraries(sdf2pov gazebo_common gazebo_sdf_interface)
#gz_install_executable(sdf2pov)

add_executable(gzfactory gzfactory.cc)
target_link_libraries(gzfactory gazebo_common gazebo_transport ${Boost_LIBRARIES} pthread)
gz_install_executable(gzfactory)

install (FILES gzmodel_create.sh DESTINATION ${BIN_INSTALL_DIR})
