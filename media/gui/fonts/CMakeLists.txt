set (files Font.xsd
           baemuk.txt
           DejaVuSans-10.font
           DejaVuSans.ttf
           FairChar-30.font
           fkp-16.font
           fkp.de.pcf
)


install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/gui/fonts/)

