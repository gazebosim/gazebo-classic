if (WIN32)
  add_definitions(-DBUILDING_DLL -DBUILDING_DLL_GZ_PLUGIN)
endif()

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
)

# Tinyxml are used since the plugins are linking against gazebo_common
# which uses tinxyml. Not defining the link directory here will end up
# failures in clang.
link_directories(
  ${CMAKE_CURRENT_BINARY_DIR}
  ${CURL_LIBDIR}
  ${CCD_LIBRARY_DIRS}
  ${SDFormat_LIBRARY_DIRS}
  ${tinyxml_LIBRARY_DIRS}
  ${IGNITION-MATH_LIBRARY_DIRS}
)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

if (HAVE_DART)
  link_directories(${DARTCore_LIBRARY_DIRS})
endif()

if (HAVE_UUID)
  include_directories(${uuid_INCLUDE_DIRS})
  link_directories(${uuid_LIBRARY_DIRS})
  add_definitions(${uuid_CFLAGS})
endif()

set (plugins_single_header
  ActorPlugin
  ActuatorPlugin
  ArduCopterPlugin
  ArrangePlugin
  BlinkVisualPlugin
  BreakableJointPlugin
  BuoyancyPlugin
  CameraPlugin
  CartDemoPlugin
  CessnaPlugin
  ContactPlugin
  DepthCameraPlugin
  DiffDrivePlugin
  FiducialCameraPlugin
  FollowerPlugin
  ForceTorquePlugin
  GimbalSmall2dPlugin
  GpuRayPlugin
  HarnessPlugin
  ImuSensorPlugin
  InitialVelocityPlugin
  JointTrajectoryPlugin
  LiftDragPlugin
  LinearBatteryConsumerPlugin
  LinearBatteryPlugin
  ModelPropShop
  MudPlugin
  PlaneDemoPlugin
  PressurePlugin
  RayPlugin
  RaySensorNoisePlugin
  RubblePlugin
  SkidSteerDrivePlugin
  SonarPlugin
  SphereAtlasDemoPlugin
  VehiclePlugin
  WindPlugin
)

set (plugins_private_header
  ElevatorPlugin
  RandomVelocityPlugin
  TransporterPlugin
)

if (HAVE_USB AND UNIX AND NOT APPLE)
  set (plugins_single_header ${plugins_single_header}
    HydraPlugin
    HydraDemoPlugin
  )
endif()

set (GUIplugins
  CessnaGUIPlugin
  KeyboardGUIPlugin
  TimerGUIPlugin
)

set(GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_MAJOR_VERSION}/plugins/
)

foreach (src ${plugins_single_header})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo ${ogre_libraries})
  install (TARGETS ${src} DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" ${src}.hh)
endforeach ()

foreach (src ${plugins_private_header})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo ${ogre_libraries})
  install (TARGETS ${src} DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" ${src}.hh)
  gz_install_includes("plugins" ${src}Private.hh)
endforeach ()

foreach (src ${GUIplugins})
  QT4_WRAP_CPP(${src}_MOC ${src}.hh)
  add_library(${src} SHARED ${src}.cc ${${src}_MOC})
  target_link_libraries(${src} libgazebo ${ogre_libraries} ${QT_LIBRARIES})
  install (TARGETS ${src} DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" ${src}.hh)
endforeach ()

add_subdirectory(events)

if (NOT WIN32)
  add_subdirectory(rest_web)
endif()
