include_directories(
  ${PROTOBUF_INCLUDE_DIR}
)

if(HAVE_URDFDOM_HEADERS)
  link_directories(
    ${urdfdom_LIBRARY_DIRS}
    ${urdfdom_headers_LIBRARY_DIRS})
endif(HAVE_URDFDOM_HEADERS)

if(HAVE_CONSOLE_BRIDGE)
  link_directories(
    ${console_bridge_LIBRARY_DIRS})
endif(HAVE_CONSOLE_BRIDGE)

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
