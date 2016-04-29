if (WIN32)
    add_definitions(-DBUILDING_DLL)
endif()

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set (validation_src
  ValidationController.cc
  ValidationPlugin.cc
)

set (validation_headers
  State.hh
  ValidationController.hh
  ValidationPlugin.hh
)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_PLUGIN_PATH})
add_library(ValidationPlugin SHARED ${validation_src})
target_link_libraries(ValidationPlugin ${GAZEBO_libraries} gazebo_msgs ${IGNITION-TRANSPORT_LIBRARIES} gazebo_physics)
install (TARGETS ValidationPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins/validation" ${validation_headers})
