set (files empty.world
           pioneer2dx.world
           pr2.world
           shapes.world
           willowgarage.world
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/worlds/)
