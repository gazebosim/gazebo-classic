set (sources Param.cc)
set (headers SDF.hh)

add_library( gazebo_sdf_interface SHARED ${sources})
target_link_libraries(gazebo_sdf_interface gazebo_common)

INSTALL_LIBRARY(gazebo_sdf_interface)
INSTALL_INCLUDES("sdf/interface" ${headers})
