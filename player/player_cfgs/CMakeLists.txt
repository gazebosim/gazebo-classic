set (cfgs gazebo.cfg 
          epuck.cfg 
          epuck_single.cfg 
          pioneerarm.cfg
    )

install(FILES ${cfgs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/player_cfgs )
