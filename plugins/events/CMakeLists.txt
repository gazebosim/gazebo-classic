link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  )

set (src
  SimEventsPlugin.cc
  EventSource.cc
  InRegionEventSource.cc
  ExistenceEventSource.cc
  SimStateEventSource.cc
  )

set (inc
  SimEventsPlugin.hh
  EventSource.hh
  InRegionEventSource.hh
  ExistenceEventSource.hh
  SimEventsException.hh
  SimStateEventSource.hh
  )

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  )

add_library(SimEventsPlugin SHARED ${src} )
target_link_libraries(SimEventsPlugin ${GAZEBO_libraries} gazebo_msgs)
install (TARGETS SimEventsPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins" ${inc})
