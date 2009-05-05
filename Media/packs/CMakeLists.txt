SET (files cubemapsJS.zip
           cubemap.zip
           dragon.zip
           fresneldemo.zip
           OgreCore.zip
           ogretestmap.zip
           skybox.zip
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/packs)
