set (files pioneer2dx.model
           pr2.model
           box.model
           willowgarage.model
           table.model
           bookshelf.model
           bowl.model
           saucepan.model
           female_standing.model
           cylinder.model
           ground_plane.model
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/)
