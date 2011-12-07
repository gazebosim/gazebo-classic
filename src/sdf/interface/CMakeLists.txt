set (sources Param.cc SDF.cc)
set (headers SDF.hh Param.hh)

add_library( gazebo_sdf_interface SHARED ${sources})
target_link_libraries(gazebo_sdf_interface gazebo_common)

INSTALL_LIBRARY(gazebo_sdf_interface)
INSTALL_INCLUDES("sdf/interface" ${headers})
