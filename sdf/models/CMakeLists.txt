set (files pioneer2dx.model
           pr2.model
           box.model
           willowgarage.model
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/)
