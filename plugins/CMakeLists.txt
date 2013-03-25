include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}/gazebo
  ${PROTOBUF_INCLUDE_DIR}
  ${SDF_INCLUDE_DIRS}
)

set (plugins
  CameraPlugin
  DepthCameraPlugin
  GpuRayPlugin
  ContactPlugin
  CartTestPlugin
  ForceTorquePlugin
  ModelTrajectoryTestPlugin
  RayPlugin
  RubblePlugin
  SpringTestPlugin
  DiffDrivePlugin
  SkidSteerDrivePlugin
  JointTrajectoryPlugin
  VehiclePlugin)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors)
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
  gz_install_includes("plugins" ${src}.hh)
endforeach (src ${plugins})
