set (sources Link.cc 
             Model.cc 
             World.cc
             Param.cc
)

set (headers Joint.hh
             Link.hh
             Model.hh
             Physics.hh
             Plugin.hh
             Scene.hh
             Sensor.hh
             World.hh 
             Param.hh
             Light.hh
             sdf.h
)

add_library( gazebo_sdf_interface SHARED ${sources})
target_link_libraries(gazebo_sdf_interface gazebo_common)

INSTALL_LIBRARY(gazebo_sdf_interface)
INSTALL_INCLUDES("sdf/interface" ${headers})
