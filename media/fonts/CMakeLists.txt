SET (files arial.ttf
           console.ttf
           font_arial.png
           font_matisse_itc.png
           Gazebo.fontdef
           maiden.ttf
           sample.fontdef
           solo5.ttf
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/fonts/)
