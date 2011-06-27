include_directories( 
  ${PROJECT_SOURCE_DIR}/src 
  ${tinyxml_include_dirs}
  ${XML_INCLUDE_DIRS}
)
link_directories(${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

add_executable( check_desc check_desc.cc )
target_link_libraries(check_desc ${tinyxml_libraries} gazebo_sdf_parser)

#add_executable( check_sdf_deprecated check_sdf_deprecated.cpp )
#target_link_libraries(check_sdf_deprecated ${XML_LIBRARIES} gazebo_sdf_parser_deprecated)
