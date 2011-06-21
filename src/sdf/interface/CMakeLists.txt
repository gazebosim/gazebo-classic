set (sources link.cc 
             model.cc 
             world.cc
             Param.cc
)

set (headers color.hh
             joint.hh
             link.hh
             model.hh
             physics.hh
             plugin.hh
             pose.hh
             scene.hh
             sensor.hh
             world.hh 
             Param.hh
)

add_library( gazebo_sdf_interface SHARED ${sources})
target_link_libraries(gazebo_sdf_interface gazebo_common)

INSTALL_LIBRARY(gazebo_sdf_interface)
INSTALL_INCLUDES("sdf/interface" ${headers})
