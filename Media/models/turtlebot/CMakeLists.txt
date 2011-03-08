set (files plate_0.tga
           plate_0.dae
           plate_1.tga
           plate_1.dae
           plate_2.tga
           plate_2.dae
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/Media/models/turtlebot)
