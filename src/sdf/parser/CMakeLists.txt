include_directories(${tinyxml_include_dirs})
link_directories( ${tinyxml_library_dirs})

set (sources parser.cc)
set (headers parser.hh)

add_library( gazebo_sdf_parser SHARED ${sources})
target_link_libraries(gazebo_sdf_parser gazebo_sdf_interface ${tinyxml_libraries})
add_definitions(${tinyxml_cflags})

INSTALL_LIBRARY(gazebo_sdf_parser)
INSTALL_INCLUDES("sdf/parser" ${headers})
