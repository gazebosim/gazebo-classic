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

###############################################################
#TODO cuda path is included hardcoded
include("/usr/share/cmake-2.8/Modules/FindCUDA.cmake")
if(CUDA_FOUND AND FLUIDIX_FOUND)
  #include the fluidix files
  include_directories(/opt/fluidix/include) # ${CUDA_INCLUDE_DIRS})

  # unhide symbols
  # Windows and MacosX does not handled properly the hidden compilation
  set (CMAKE_CXX_FLAGS_BAK "${CMAKE_CXX_FLAGS}") # backup flags

  set (CMAKE_CXX_FLAGS "-Wall -Wextra -Wno-long-long -Wno-unused-value -Wno-unused-value
                        -Wno-unused-value -Wno-unused-value -Wfloat-equal -Wshadow
                        -Winit-self -Wswitch-default -Wmissing-include-dirs -pedantic")

  set (CMAKE_CXX_FLAGS "-Wno-unused-value")

	#SET(CUDA_NVCC_FLAGS "-arch;sm_30 -use_fast_math -lm -ldl -lrt -Xcompiler \"-fPIC\"")
	SET(CUDA_NVCC_FLAGS "-arch=sm_30 -Iinclude -use_fast_math -lm -ldl -lrt -Xcompiler=\"-fPIC\" -Xcompiler=\"-O3\" -Xcompiler=\"-fexpensive-optimizations\" -Xcompiler=\"-funroll-loops\" -Xcompiler=\"-ffast-math\"")
	#SET(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_20,code=sm_20)

	# compile CUDA library
	cuda_add_library(FluidEngine SHARED FluidEngine.cu FluidSet.cc FluidObjectSet.cc)
  install (TARGETS FluidEngine DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" FluidEngine.hh)

	# compile client plugin
  add_library(FluidVisPlugin SHARED FluidVisPlugin.cc)
  target_link_libraries(FluidVisPlugin gazebo_msgs ${OGRE_LIBRARIES})
  set_target_properties(FluidVisPlugin PROPERTIES INSTALL_RPATH ${GAZEBO_PLUGIN_INSTALL_DIR})
  install (TARGETS FluidVisPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" FluidVisPlugin.hh)

	# compile server plugin
  add_library(FluidWorldPlugin SHARED FluidWorldPlugin.cc)
  target_link_libraries(FluidWorldPlugin gazebo_msgs FluidEngine ${GAZEBO_libraries})
  set_target_properties(FluidWorldPlugin PROPERTIES INSTALL_RPATH ${GAZEBO_PLUGIN_INSTALL_DIR})
  # set_target_properties(FluidWorldPlugin PROPERTIES INSTALL_RPATH /opt/fluidix/lib)
  install (TARGETS FluidWorldPlugin DESTINATION ${GAZEBO_PLUGIN_INSTALL_DIR})
  gz_install_includes("plugins" FluidWorldPlugin.hh)

  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_BAK}") # recover flags

else(CUDA_FOUND)
    message("CUDA is not installed on this system.")
endif()
###############################################################

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
