set (files pioneer2dx.model
           pr2.model
           pr2_visual_only.model
           box.model
           willowgarage.model
           table.model
           bookshelf.model
           bowl.model
           saucepan.model
           female_standing.model
           cylinder.model
           ground_plane.model
           youbot.model
           warehouse_interior.model
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/)
