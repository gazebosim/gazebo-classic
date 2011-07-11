SET (files Gazebo.material 
           shadow_receiver.program 
           shadow_caster.program
           perpixel.program
           blur.compositor
           blur.material
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/materials/scripts/)
