set (files
           kitchen.dae
           texture0.jpg
           texture1.jpg
           texture2.jpg
           texture3.jpg
           texture4.jpg
           texture5.jpg
           kitchen_walls.dae
           wall.tga
           wall_Cube.tga
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/models/kitchen)

