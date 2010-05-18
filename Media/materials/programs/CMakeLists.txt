SET (files PointLight.vert 
           PointLight.frag 
           DepthMap.frag 
           DepthMap.vert 
           )

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/materials/programs/)

