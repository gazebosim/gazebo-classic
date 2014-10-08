set (files handsim.png
           GUIAratPlugin.sdf
          
)


install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/gui/etc/)

