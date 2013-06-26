include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR}/gazebo
  ${PROTOBUF_INCLUDE_DIR}
  ${SDF_INCLUDE_DIRS}
)

# Remove this whole if block in Gazebo 1.8
if (NOT HAVE_SDF)
  link_directories(
    ${urdfdom_LIBRARY_DIRS}
    ${urdfdom_headers_LIBRARY_DIRS}
    ${console_bridge_LIBRARY_DIRS}
    )
endif()


set (plugins
  CameraPlugin
  CartTestPlugin
  ContactPlugin
  DepthCameraPlugin
  DiffDrivePlugin
  ForceTorquePlugin
  GpuRayPlugin
  JointTrajectoryPlugin
  ModelTrajectoryTestPlugin
  MudPlugin
  PressurePlugin
  RayPlugin
  RubblePlugin
  SkidSteerDrivePlugin
  SonarPlugin
  SpringTestPlugin
  VehiclePlugin)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors)
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
  gz_install_includes("plugins" ${src}.hh)
endforeach (src ${plugins})
