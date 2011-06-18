include_directories(${tinyxml_include_dirs})
link_directories(${tinyxml_library_dirs})

add_library( convert  link.cpp model.cpp parser.cpp world.cpp)
target_link_libraries(convert ${tinyxml_libraries})

add_executable( check check_sdf.cpp )
target_link_libraries(check ${tinyxml_libraries} convert)

add_definitions(${tinyxml_cflags})
