set (files Falagard.xsd
           OgreTray.looknfeel
           TaharezLook.looknfeel
           Vanilla.looknfeel
           WindowsLook.looknfeel
           Gazebo.looknfeel
)


install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/gui/looknfeel/)

