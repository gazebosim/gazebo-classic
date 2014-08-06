include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

# Tinyxml are used since the plugins are linking against gazebo_common
# which uses tinxyml. Not defining the link directory here will end up
# failures in clang.
link_directories(
  ${SDFormat_LIBRARY_DIRS}
  ${tinyxml_LIBRARY_DIRS}
)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

set (plugins
  BreakableJointPlugin
  CameraPlugin
  CartDemoPlugin
  ContactPlugin
  DepthCameraPlugin
  DiffDrivePlugin
  ForceTorquePlugin
  GpuRayPlugin
  JointTrajectoryPlugin
  LiftDragPlugin
  ModelPropShop
  MudPlugin
  PressurePlugin
  RayPlugin
  RaySensorNoisePlugin
  RubblePlugin
  SkidSteerDrivePlugin
  SonarPlugin
  SphereAtlasDemoPlugin
  VehiclePlugin)

if (HAVE_USB AND UNIX AND NOT APPLE)
  set (plugins ${plugins}
    HydraPlugin
    HydraDemoPlugin
  )
endif()

set(GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/
)
foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors ${ogre_libraries})
  install (TARGETS ${src} DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" ${src}.hh)
endforeach (src ${plugins})

target_link_libraries(BreakableJointPlugin ForceTorquePlugin)
set_target_properties(BreakableJointPlugin PROPERTIES
  INSTALL_RPATH ${GAZEBO_PLUGIN_INSTALL_DIR}
)
