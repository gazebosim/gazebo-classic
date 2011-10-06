set (files pioneer2dx.model
           pr2.model
           box.model
           willowgarage.model
           table.model
           bookshelf.model
           kitchen_counter.model
           man_standing.model
           bowl.model
           saucepan.model
           kitchen.model
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/models/)
