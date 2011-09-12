set (files stovetop.dae
           stovetop.stl
           texture0.jpg
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/models/stovetop)
