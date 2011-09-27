set (files Falagard.xsd
           OgreTray.looknfeel
           TaharezLook.looknfeel
           Vanilla.looknfeel
           WindowsLook.looknfeel
)


install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/gui/looknfeel/)

