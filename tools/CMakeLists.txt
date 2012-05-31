include_directories( 
  ${PROJECT_BINARY_DIR}/src 
  ${PROJECT_SOURCE_DIR}/src 
  ${TINYXML_INCLUDE_DIRS}
  ${XML_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

link_directories(${TINYXML_LIBRARY_DIRS})
add_definitions(${TINYXML_CFLAGS})

add_executable(gzsdf gzsdf.cc)
target_link_libraries(gzsdf ${TINYXML_LIBRARIES} gazebo_sdf_interface)
INSTALL_EXECUTABLE(gzsdf)

add_executable(gztopic gztopic.cc)
target_link_libraries(gztopic gazebo_msgs gazebo_common gazebo_transport )
INSTALL_EXECUTABLE(gztopic)

add_executable(gzstats gzstats.cc)
target_link_libraries(gzstats gazebo_msgs gazebo_common gazebo_transport )
INSTALL_EXECUTABLE(gzstats)

add_executable(sdf2pov sdf2pov.cc)
target_link_libraries(sdf2pov gazebo_common gazebo_sdf_interface)
INSTALL_EXECUTABLE(sdf2pov)

add_executable(gzfactory gzfactory.cc)
target_link_libraries(gzfactory gazebo_common gazebo_transport ${Boost_LIBRARIES})
INSTALL_EXECUTABLE(gzfactory)
