include_directories(${tinyxml_include_dirs})
link_directories(${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

set (sources link.cc 
             model.cc 
             world.cc
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
)

add_library( gazebo_sdf_interface SHARED ${sources})
target_link_libraries(gazebo_sdf_interface ${tinyxml_libraries} gazebo_common)

INSTALL_LIBRARY(gazebo_sdf_interface)
INSTALL_INCLUDES("sdf/interface" ${headers})
