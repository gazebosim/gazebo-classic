set (files Font.xsd
           DejaVuSans-10.font
           DejaVuSans.ttf
           FairChar-30.font
           fkp-16.font
           fkp.de.pcf
)


install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/media/gui/fonts/)

