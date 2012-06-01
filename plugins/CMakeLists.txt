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
  ModelTrajectoryTestPlugin
  RayPlugin
  SpringTestPlugin
  DiffDrivePlugin
  VehiclePlugin)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors)
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo-${GAZEBO_VERSION}/plugins/)
  gz_install_includes("plugins" ${src}.hh)
endforeach (src ${plugins})
