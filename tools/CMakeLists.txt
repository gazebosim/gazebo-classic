include_directories( 
  ${PROJECT_BINARY_DIR}/gazebo 
  ${PROJECT_SOURCE_DIR}/gazebo 
  ${TINYXML_INCLUDE_DIRS}
  ${XML_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

link_directories(${TINYXML_LIBRARY_DIRS})
add_definitions(${TINYXML_CFLAGS})

add_executable(gzsdf gzsdf.cc)
target_link_libraries(gzsdf ${TINYXML_LIBRARIES} gazebo_sdf_interface)
gz_install_executable(gzsdf)

add_executable(gztopic gztopic.cc)
target_link_libraries(gztopic gazebo_msgs gazebo_common gazebo_transport )
gz_install_executable(gztopic)

add_executable(gzstats gzstats.cc)
target_link_libraries(gzstats gazebo_msgs gazebo_common gazebo_transport )
gz_install_executable(gzstats)

add_executable(sdf2pov sdf2pov.cc)
target_link_libraries(sdf2pov gazebo_common gazebo_sdf_interface)
gz_install_executable(sdf2pov)

add_executable(gzfactory gzfactory.cc)
target_link_libraries(gzfactory gazebo_common gazebo_transport ${Boost_LIBRARIES})
gz_install_executable(gzfactory)
