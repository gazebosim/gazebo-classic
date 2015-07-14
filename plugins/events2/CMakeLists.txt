link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set (src
  RegionEventBoxPlugin.cc
)

set (inc
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(RegionEventBoxPlugin SHARED ${src} )
install (TARGETS RegionEventBoxPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins/events" ${inc})
