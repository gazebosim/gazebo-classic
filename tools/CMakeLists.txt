include_directories( 
  ${PROJECT_BINARY_DIR}/src 
  ${PROJECT_SOURCE_DIR}/src 
  ${tinyxml_include_dirs}
  ${XML_INCLUDE_DIRS}
)

link_directories(${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

add_executable( gzsdf check_sdf.cc )
target_link_libraries(gzsdf ${tinyxml_libraries} gazebo_sdf_parser)
INSTALL_EXECUTABLE( gzsdf )

add_executable( gztopic gztopic.cc )
target_link_libraries(gztopic gazebo_msgs gazebo_common gazebo_transport )
INSTALL_EXECUTABLE( gztopic )

add_executable( gzstats gzstats.cc )
target_link_libraries(gzstats gazebo_msgs gazebo_common gazebo_transport )
INSTALL_EXECUTABLE( gzstats )

