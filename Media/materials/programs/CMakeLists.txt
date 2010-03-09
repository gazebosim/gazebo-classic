#SET (files AmbientOneTexture.frag
#           AmbientOneTexture.vert
#           DepthMap.frag
#           DepthMap.vert
#           DirectionalLight.frag
#           DirectionalLight.vert
#           PointLight.frag
#           PointLight.vert
#           SpotLight.frag
#           SpotLight.vert
#           Ambient.vert
#           Ambient.frag
#)

SET (files PointLight.vert PointLight.frag DepthMap.frag DepthMap.vert)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/materials/programs/)
