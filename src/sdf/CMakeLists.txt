add_subdirectory(interface)
add_subdirectory(parser)

set (headers sdf.h sdf_parser.h)
set (sdfs gazebo.sdf 
          collision.sdf 
          geometry.sdf 
          inertial.sdf 
          joint.sdf 
          light.sdf 
          link.sdf 
          model.sdf 
          physics.sdf 
          plugin.sdf 
          scene.sdf 
          sensor.sdf 
          surface.sdf 
          visual.sdf 
          world.sdf
          camera.sdf
          ray.sdf
          contact.sdf
)

INSTALL_INCLUDES("sdf" ${headers})

INSTALL(FILES ${sdfs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/sdf/)
