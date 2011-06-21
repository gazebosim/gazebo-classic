include_directories(${XML_INCLUDE_DIRS})

set (sources parser_deprecated.hh)
set (headers parser_deprecated.hh)

add_library( gazebo_sdf_parser_deprecated SHARED ${sources})
target_link_libraries(gazebo_sdf_parser_deprecated gazebo_sdf_interface ${XML_LIBRARIES})

INSTALL_LIBRARY(gazebo_sdf_parser_deprecated)
INSTALL_INCLUDES("sdf/parser_deprecated" ${headers})
 
