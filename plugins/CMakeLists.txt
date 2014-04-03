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
  BreakableJointPlugin
  CameraPlugin
  CartDemoPlugin
  ContactPlugin
  DepthCameraPlugin
  DiffDrivePlugin
  DRCVehiclePlugin
  ForceTorquePlugin
  GpuRayPlugin
  Hydra
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
  
add_library(AtlasPlugin SHARED AtlasPlugin.cc)
target_link_libraries(AtlasPlugin libgazebo gazebo_sensors ${ogre_libraries} ${Orocos-KDL_LIBRARIES})
install (TARGETS AtlasPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
gz_install_includes("plugins" AtlasPlugin.hh)

add_library(SimpleIKPlugin SHARED SimpleIKPlugin.cc)
target_link_libraries(SimpleIKPlugin libgazebo gazebo_sensors ${ogre_libraries} ${Orocos-KDL_LIBRARIES})
install (TARGETS SimpleIKPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
gz_install_includes("plugins" SimpleIKPlugin.hh)


add_library(RobonautPlugin SHARED RobonautPlugin.cc)
target_link_libraries(RobonautPlugin libgazebo gazebo_sensors ${ogre_libraries} ${Orocos-KDL_LIBRARIES})
install (TARGETS RobonautPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
gz_install_includes("plugins" RobonautPlugin.hh)

add_library(SimpleArmPlugin SHARED SimpleArmPlugin.cc)
target_link_libraries(SimpleArmPlugin libgazebo gazebo_sensors ${ogre_libraries} ${Orocos-KDL_LIBRARIES})
install (TARGETS SimpleArmPlugin DESTINATION ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_VERSION}/plugins/)
gz_install_includes("plugins" SimpleArmPlugin.hh)
