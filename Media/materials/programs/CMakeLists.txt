SET (files shadow_caster_fp.glsl
           shadow_caster_vp.glsl
           spot_shadow_receiver_fp.glsl
           spot_shadow_receiver_vp.glsl
           directional_shadow_receiver_fp.glsl
           directional_shadow_receiver_vp.glsl
           point_receiver_vp.glsl
           point_receiver_fp.glsl
           ambient_one_texture_vp.glsl
           blur.glsl
           perpixel_vp.glsl
           perpixel_fp.glsl
) 

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/materials/programs/)


