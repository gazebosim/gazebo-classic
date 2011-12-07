set (files texture0.jpg
           texture1.jpg
           texture2.jpg
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/models/wg)
