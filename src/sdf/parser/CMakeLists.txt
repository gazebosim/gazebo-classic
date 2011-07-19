include_directories(${tinyxml_include_dirs} ${XML_INCLUDE_DIRS})
link_directories( ${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

set (sources parser.cc parser_deprecated.cc)
set (headers parser.hh parser_deprecated.hh)

add_library( gazebo_sdf_parser SHARED ${sources})
target_link_libraries(gazebo_sdf_parser gazebo_sdf_interface ${tinyxml_libraries} ${XML_LIBRARIES})

INSTALL_LIBRARY(gazebo_sdf_parser)
INSTALL_INCLUDES("sdf/parser" ${headers})
