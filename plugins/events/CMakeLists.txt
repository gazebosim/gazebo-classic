if (WIN32)
    add_definitions(-DBUILDING_DLL)
endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set (sim_event_src
  EventSource.cc
  ExistenceEventSource.cc
  InRegionEventSource.cc
  JointEventSource.cc
  OccupiedEventSource.cc
  Region.cc
  SimEventsPlugin.cc
  SimStateEventSource.cc
)

set (sim_event_include
  EventSource.hh
  ExistenceEventSource.hh
  InRegionEventSource.hh
  JointEventSource.hh
  OccupiedEventSource.hh
  Region.hh
  SimEventsException.hh
  SimEventsPlugin.hh
  SimStateEventSource.hh
)

set (src
  EventSource.cc
  RegionEventBoxPlugin.cc
)

set (inc
  RegionEventBoxPlugin.hh
)


include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(SimEventsPlugin SHARED ${sim_event_src})
target_link_libraries(SimEventsPlugin ${GAZEBO_libraries} gazebo_msgs)
install (TARGETS SimEventsPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins/events" ${sim_event_include})

add_library(RegionEventBoxPlugin SHARED ${src})
target_link_libraries(RegionEventBoxPlugin ${GAZEBO_libraries} gazebo_msgs)
install (TARGETS RegionEventBoxPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins/events" ${inc})
