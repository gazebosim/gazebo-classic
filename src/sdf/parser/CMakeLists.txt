set (sources parser.cc)
set (headers parser.hh)

add_library( gazebo_sdf_parser SHARED ${sources})
target_link_libraries(gazebo_sdf_parser gazebo_sdf_interface)

INSTALL_LIBRARY(gazebo_sdf_parser)
INSTALL_INCLUDES("sdf/parser" ${headers})
 
