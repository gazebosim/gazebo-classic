include_directories(${TINY_XML_INCLUDE_DIRS})

add_library( convert  joint.cpp link.cpp model.cpp sensor.cpp parser.cpp plugin.cpp world.cpp scene.cpp physics.cpp)
target_link_libraries(convert ${TINY_XML_LIBRARIES})

add_executable( check check_sdf.cpp )
target_link_libraries(check ${TINY_XML_LIBRARIES} convert)
