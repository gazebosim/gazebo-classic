set (files pioneer2dx.model
           pr2.model
           pr2_visual_only.model
           box.model
           willowgarage.model
           willowgarage_visual.model
           table.model
           bookshelf.model
           bowl.model
           saucepan.model
           female_standing.model
           cylinder.model
           ground_plane.model
           youbot.model
           youbot_static.model
           quadrotor.model
           weshare2012_apartment.model
           weshare2012_pioneer.model
           weshare2012_bookshelf.model
           kitchen.model
           hokuyo.model
           camera.model
           sun.light
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/)
