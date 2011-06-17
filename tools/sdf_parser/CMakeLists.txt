include_directories(${TINY_XML_INCLUDE_DIRS})

add_library( convert  link.cpp model.cpp parser.cpp world.cpp)
target_link_libraries(convert ${TINY_XML_LIBRARIES})

add_executable( check check_sdf.cpp )
target_link_libraries(check ${TINY_XML_LIBRARIES} convert)
