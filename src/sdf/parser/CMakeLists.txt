include_directories(${tinyxml_include_dirs})
link_directories( ${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

set (sources format_parser.cc)
set (headers format_parser.hh)

add_library( gazebo_sdf_parser SHARED ${sources})
target_link_libraries(gazebo_sdf_parser gazebo_sdf_interface ${tinyxml_libraries})

INSTALL_LIBRARY(gazebo_sdf_parser)
INSTALL_INCLUDES("sdf/parser" ${headers})
