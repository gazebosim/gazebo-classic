link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

set (common_inc
  DistributedSimulationPrivate.hh
)

set (manager_inc
  DistributedSimulationManager.hh
  )

set (manager_src
  DistributedSimulationManager.cc
  )

set (client_inc
  DistributedSimulationClient.hh
  )

set (client_src
  DistributedSimulationClient.cc
  )

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(DistributedManager SHARED ${manager_src} )
target_link_libraries(DistributedManager ${GAZEBO_libraries} gazebo_msgs)
install (TARGETS DistributedManager DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})

add_library(DistributedClient SHARED ${client_src} )
target_link_libraries(DistributedClient ${GAZEBO_libraries} gazebo_msgs)
install (TARGETS DistributedClient DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})


gz_install_includes("plugins" ${manager_inc} ${common_inc} ${client_inc})
