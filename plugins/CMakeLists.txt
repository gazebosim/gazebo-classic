include_directories(
  ${PROJECT_BINARY_DIR}/gazebo 
  ${PROJECT_SOURCE_DIR}/gazebo 
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
  RubblePlugin
  SpringTestPlugin
  DiffDrivePlugin
  JointTrajectoryPlugin
  VehiclePlugin)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors)
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
  gz_install_includes("plugins" ${src}.hh)
endforeach (src ${plugins})
