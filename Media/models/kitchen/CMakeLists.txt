set (files
           kitchen.dae
           texture0.jpg
           texture1.jpg
           texture2.jpg
           texture3.jpg
           texture4.jpg
           texture5.jpg
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/models/kitchen)

