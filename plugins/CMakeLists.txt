include_directories(
  ${PROJECT_BINARY_DIR}/src 
  ${PROJECT_SOURCE_DIR}/src 
  ${PROTOBUF_INCLUDE_DIR}
)

set (plugins
  CameraPlugin
  DepthCameraPlugin
  GpuRayPlugin
  ContactPlugin
  CartTestPlugin
  RayPlugin
  DiffDrivePlugin
  GripperPlugin
  VehiclePlugin)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors)
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo-${GAZEBO_VERSION}/plugins/)
  INSTALL_INCLUDES("plugins" ${src}.hh)
endforeach (src ${plugins})
