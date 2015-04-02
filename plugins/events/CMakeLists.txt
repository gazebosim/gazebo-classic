link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set (src
  EventSource.cc
  ExistenceEventSource.cc
  InRegionEventSource.cc
  SimEventsPlugin.cc
  SimStateEventSource.cc
)

set (inc
  EventSource.hh
  ExistenceEventSource.hh
  InRegionEventSource.hh
  SimEventsException.hh
  SimEventsPlugin.hh
  SimStateEventSource.hh
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(SimEventsPlugin SHARED ${src} )
target_link_libraries(SimEventsPlugin ${GAZEBO_libraries} gazebo_msgs)
install (TARGETS SimEventsPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins" ${inc})
