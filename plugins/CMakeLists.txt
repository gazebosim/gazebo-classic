include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDF_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${Orocos-KDL_INCLUDE_DIRS}
)

link_directories(
  ${SDF_LIBRARY_DIRS}
  ${Orocos-KDL_LIBRARY_DIRS}
)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

set (plugins
  CameraPlugin
  CartTestPlugin
  ContactPlugin
  DepthCameraPlugin
  DiffDrivePlugin
  DRCVehiclePlugin
  ForceTorquePlugin
  GpuRayPlugin
  Hydra
  JointTrajectoryPlugin
  ModelPropShop
  ModelTrajectoryTestPlugin
  MudPlugin
  PressurePlugin
  RayPlugin
  RubblePlugin
  SkidSteerDrivePlugin
  SonarPlugin
  SpringTestPlugin
  SphereAtlasTestPlugin
  VehiclePlugin)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors ${ogre_libraries})
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
  gz_install_includes("plugins" ${src}.hh)
endforeach (src ${plugins})

  
add_library(AtlasPlugin SHARED AtlasPlugin.cc)
target_link_libraries(AtlasPlugin libgazebo gazebo_sensors ${ogre_libraries} ${Orocos-KDL_LIBRARIES})
install (TARGETS AtlasPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
gz_install_includes("plugins" AtlasPlugin.hh)

add_library(SimpleIKPlugin SHARED SimpleIKPlugin.cc)
target_link_libraries(SimpleIKPlugin libgazebo gazebo_sensors ${ogre_libraries} ${Orocos-KDL_LIBRARIES})
install (TARGETS SimpleIKPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
gz_install_includes("plugins" SimpleIKPlugin.hh)

