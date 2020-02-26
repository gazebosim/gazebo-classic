if (WIN32)
  add_definitions(-DBUILDING_DLL -DBUILDING_DLL_GZ_PLUGIN -DNOUSER)
endif()

if(WIN32 AND NOT UNIX)
  # Solves errors about not finding _imp_sprintf and similar on some Windows systems.
  set(CMAKE_CXX_STANDARD_LIBRARIES "${CMAKE_CXX_STANDARD_LIBRARIES} legacy_stdio_definitions.lib")

  # Include Universal CRT includes. They are sometimes needed by gtest. If not added,
  # compilation fails not finding symbols like _isatty, write, close etc.
  get_filename_component(windows_kits_dir
    "[HKEY_LOCAL_MACHINE\\SOFTWARE\\Microsoft\\Windows Kits\\Installed Roots;KitsRoot10]" ABSOLUTE)
  set(programfilesx86 "ProgramFiles(x86)")  
  find_path(WINDOWS_KITS_UCRT_INCLUDE NAMES assert.h
    PATHS
    "${windows_kits_dir}/include"
    "$ENV{ProgramFiles}/Windows Kits/10/include"
    "$ENV{${programfilesx86}}/Windows Kits/10/include"
  )  

  message(STATUS "Found Windows SDK Universal CRT include directory: ${WINDOWS_KITS_UCRT_INCLUDE}")
  include_directories(${WINDOWS_KITS_UCRT_INCLUDE})  
endif()

include_directories(SYSTEM
  ${CURL_INCLUDEDIR}
  ${PROJECT_SOURCE_DIR}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
)

include_directories(
  ${PROJECT_SOURCE_DIR}
)

# needed for SimpleTrackedVehiclePlugin
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${CMAKE_SOURCE_DIR}/deps/opende/include
  ${CMAKE_SOURCE_DIR}/deps/threadpool
  ${CMAKE_SOURCE_DIR}/deps/opende/src
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
)

if (WIN32)
  add_definitions(-DCURL_STATICLIB)
endif()
add_definitions(${Qt5Core_DEFINITIONS})
set (CMAKE_AUTOMOC ON)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

if (HAVE_DART)
  include_directories(${DART_INCLUDE_DIRS})
  link_directories(${DART_LIBRARY_DIRS})
endif()

if (HAVE_UUID)
  include_directories(${uuid_INCLUDE_DIRS})
  link_directories(${uuid_LIBRARY_DIRS})
  add_definitions(${uuid_CFLAGS})
endif()

set (plugins_single_header
  ActorPlugin
  ActuatorPlugin
  AmbientOcclusionVisualPlugin
  ArduCopterPlugin
  ArrangePlugin
  AttachLightPlugin
  BlinkVisualPlugin
  BreakableJointPlugin
  BuoyancyPlugin
  CameraPlugin
  CartDemoPlugin
  CessnaPlugin
  ContactPlugin
  ContainPlugin
  DepthCameraPlugin
  DiffDrivePlugin
  FiducialCameraPlugin
  FlashLightPlugin
  FollowerPlugin
  ForceTorquePlugin
  GimbalSmall2dPlugin
  GpuRayPlugin
  HarnessPlugin
  HeightmapLODPlugin
  ImuSensorPlugin
  InitialVelocityPlugin
  JointControlPlugin
  JointTrajectoryPlugin
  KeysToCmdVelPlugin
  KeysToJointsPlugin
  LedPlugin
  LensFlareSensorPlugin
  LiftDragPlugin
  LinearBatteryConsumerPlugin
  LinearBatteryPlugin
  LinkPlot3DPlugin
  MisalignmentPlugin
  ModelPropShop
  MudPlugin
  PlaneDemoPlugin
  PressurePlugin
  RayPlugin
  RaySensorNoisePlugin
  RubblePlugin
  ShaderParamVisualPlugin
  SimpleTrackedVehiclePlugin
  SkidSteerDrivePlugin
  SonarPlugin
  SphereAtlasDemoPlugin
  StaticMapPlugin
  TouchPlugin
  VariableGearboxPlugin
  VehiclePlugin
  WheelSlipPlugin
  WheelTrackedVehiclePlugin
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
    JoyPlugin
  )
endif()

set (GUIplugins
  CessnaGUIPlugin
  KeyboardGUIPlugin
  LookAtDemoPlugin
  TimerGUIPlugin
)

set (DARTplugins
  GravityCompensationPlugin
)

set(GAZEBO_PLUGIN_INSTALL_DIR
  ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/gazebo-${GAZEBO_MAJOR_VERSION}/plugins/
)

add_library(TrackedVehiclePlugin STATIC TrackedVehiclePlugin.cc)
target_link_libraries(TrackedVehiclePlugin
        libgazebo
        ${ogre_libraries}
        ${IGNITION-TRANSPORT_LIBRARIES}
        )
install (TARGETS TrackedVehiclePlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
gz_install_includes("plugins" TrackedVehiclePlugin.hh)

foreach (src ${plugins_single_header})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src}
    libgazebo
    ${ogre_libraries}
    ${IGNITION-TRANSPORT_LIBRARIES}
  )
  install (TARGETS ${src} DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" ${src}.hh)
endforeach ()

target_include_directories(SimpleTrackedVehiclePlugin SYSTEM PRIVATE
        ${CMAKE_SOURCE_DIR}/deps/opende/include)
target_link_libraries(SimpleTrackedVehiclePlugin TrackedVehiclePlugin)
add_dependencies(SimpleTrackedVehiclePlugin TrackedVehiclePlugin)

target_link_libraries(WheelTrackedVehiclePlugin TrackedVehiclePlugin)
add_dependencies(WheelTrackedVehiclePlugin TrackedVehiclePlugin)

foreach (src ${plugins_private_header})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src}
    libgazebo
    ${ogre_libraries}
    ${IGNITION-TRANSPORT_LIBRARIES}
  )
  install (TARGETS ${src} DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" ${src}.hh)
  gz_install_includes("plugins" ${src}Private.hh)
endforeach ()

foreach (src ${GUIplugins})
  add_library(${src} SHARED ${src}.cc ${${src}_MOC})
  target_link_libraries(${src}
                        libgazebo
                        gazebo_gui
                        ${ogre_libraries}
                        ${Qt5Core_LIBRARIES}
                        ${Qt5Test_LIBRARIES}
                        ${Qt5Widgets_LIBRARIES}
                        ${IGNITION-TRANSPORT_LIBRARIES}
)
  install (TARGETS ${src} DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" ${src}.hh)
endforeach ()

if (HAVE_DART_URDF)
  foreach (src ${DARTplugins})
    add_library(${src} SHARED ${src}.cc)
    target_link_libraries(${src}
      libgazebo
      ${DART_LIBRARIES}
    )
    install (TARGETS ${src} DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
    gz_install_includes("plugins" ${src}.hh)
  endforeach ()
endif()

# If a plugin depends on other plugins in the same directory, add them to link.
# Plus, set RPATH to the directory so they can dynamically link.
target_link_libraries(LedPlugin FlashLightPlugin)
set_target_properties(
  LedPlugin PROPERTIES INSTALL_RPATH ${GAZEBO_PLUGIN_INSTALL_DIR})

add_subdirectory(events)

if (NOT WIN32)
  add_subdirectory(rest_web)
endif()

# unit tests

gz_build_tests(TrackedVehiclePlugin_TEST.cc EXTRA_LIBS
  gazebo_physics
  gazebo_test_fixture
  TrackedVehiclePlugin
)

# Linking to the built dynamic library is not possible since it doesn't export
# all the symbols we need for the tests. And exporting these symbols causes
# weird segfaults...
set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS WheelTrackedVehiclePlugin.cc)
gz_build_tests(WheelTrackedVehiclePlugin_TEST.cc EXTRA_LIBS
  gazebo_physics
  gazebo_test_fixture
  TrackedVehiclePlugin
)

# Linking to the built dynamic library is not possible since it doesn't export
# all the symbols we need for the tests. And exporting these symbols causes
# weird segfaults...
set(GZ_BUILD_TESTS_EXTRA_EXE_SRCS SimpleTrackedVehiclePlugin.cc)
gz_build_tests(SimpleTrackedVehiclePlugin_TEST.cc EXTRA_LIBS
  gazebo_physics
  gazebo_test_fixture
  TrackedVehiclePlugin
)
if (HAVE_DART)
  target_include_directories(UNIT_SimpleTrackedVehiclePlugin_TEST
    PRIVATE ${DARTCore_INCLUDE_DIRS})
endif()
