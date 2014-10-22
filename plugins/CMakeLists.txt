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
  ArrangePlugin
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

# customize for haptix control plugin
find_package(haptix_comm REQUIRED)
find_package(ignition-transport REQUIRED)
message(status "debug:\n\n\n${CMAKE_CXX_FLAGS} ${IGNITION-TRANSPORT_CXX_FLAGS}\n\n\n")
# make below target specific?
include_directories(
  ${CMAKE_SOURCE_DIR}/deps/polhemus_driver/include
  ${HAPTIX_COMM_INCLUDE_DIRS}
  ${IGNITION-TRANSPORT_INCLUDE_DIRS})
# make below target specific?
link_directories(
  ${HAPTIX_COMM_LIBRARY_DIRS}
  ${IGNITION-TRANSPORT_LIBRARY_DIRS})
# build library
add_library(HaptixControlPlugin SHARED HaptixControlPlugin.cc)
# additional library dependencies
add_dependencies(HaptixControlPlugin
  gazebo_msgs
)
target_link_libraries(HaptixControlPlugin ${GAZEBO_LIBRARIES}
  spnav polhemus_driver haptix_comm
  ${IGNITION-TRANSPORT_LIBRARIES})
# below adds -std=c++11, needed to compile against ign-transport
set_target_properties(HaptixControlPlugin PROPERTIES
  COMPILE_FLAGS ${IGNITION-TRANSPORT_CXX_FLAGS})
# install library and header
install (TARGETS HaptixControlPlugin DESTINATION
  ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins" HaptixControlPlugin.hh)
