include_directories(${TINY_XML_INCLUDE_DIRS})

add_library( convert  joint.cpp link.cpp model.cpp parser.cpp sensor.cpp)
target_link_libraries(convert ${TINY_XML_LIBRARIES})

add_executable( check  check_gdf.cpp )
target_link_libraries(check ${TINY_XML_LIBRARIES} convert)
