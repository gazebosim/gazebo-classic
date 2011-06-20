include_directories( 
  ${PROJECT_SOURCE_DIR}/src 
  ${tinyxml_include_dirs}
)
link_directories(${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})


add_executable( check_sdf check_sdf.cpp )
target_link_libraries(check_sdf ${tinyxml_libraries} gazebo_sdf_parser)
