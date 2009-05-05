SET (files AmbientOneTexture.frag
           AmbientOneTexture.vert
           DepthMap.frag
           DepthMap.vert
           DirectionalLight.frag
           DirectionalLight.vert
           PointLight.frag
           PointLight.vert
           SpotLight.frag
           SpotLight.vert
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/materials/programs/)
