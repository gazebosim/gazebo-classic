set (files handsim.png
           icon_placeholder.png
           GUIAratPlugin.sdf
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/gui/arat/)

add_subdirectory(arat_icons)
