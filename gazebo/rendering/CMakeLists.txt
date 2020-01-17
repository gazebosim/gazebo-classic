include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM
  ${OGRE_INCLUDE_DIRS}
  ${TBB_INCLUDEDIR}
  skyx
  skyx/include
)

if (WIN32)
  include_directories(${libdl_include_dir})
endif()

include_directories(${tinyxml_INCLUDE_DIRS})
link_directories(${tinyxml_LIBRARY_DIRS} ${IGNITION-MSGS_LIBRARY_DIRS})

set (sources
  ApplyWrenchVisual.cc
  ArrowVisual.cc
  AxisVisual.cc
  Camera.cc
  CameraVisual.cc
  COMVisual.cc
  ContactVisual.cc
  Conversions.cc
  CustomPSSMShadowCameraSetup.cc
  DepthCamera.cc
  Distortion.cc
  DynamicLines.cc
  DynamicRenderable.cc
  FPSViewController.cc
  GpuLaser.cc
  Grid.cc
  Heightmap.cc
  InertiaVisual.cc
  JointVisual.cc
  LaserVisual.cc
  LensFlare.cc
  LinkFrameVisual.cc
  MarkerManager.cc
  MarkerVisual.cc
  SonarVisual.cc
  Light.cc
  LogicalCameraVisual.cc
  Material.cc
  MovableText.cc
  OrbitViewController.cc
  OriginVisual.cc
  OrthoViewController.cc
  Projector.cc
  RayQuery.cc
  RenderEngine.cc
  RenderEvents.cc
  RenderingIface.cc
  Road2d.cc
  RFIDVisual.cc
  RFIDTagVisual.cc
  RTShaderSystem.cc
  Scene.cc
  SelectionObj.cc
  TransmitterVisual.cc
  UserCamera.cc
  VideoVisual.cc
  ViewController.cc
  Visual.cc
  WideAngleCamera.cc
  WireBox.cc
  WindowManager.cc
  WrenchVisual.cc
)

if (NOT USE_EXTERNAL_TINYXML)
  include_directories (${CMAKE_SOURCE_DIR}/deps/win/tinyxml)
  set (sources ${sources}
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinystr.cpp
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinyxml.cpp
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinyxmlerror.cpp
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinyxmlparser.cpp)
endif()

set (headers
  ApplyWrenchVisual.hh
  ArrowVisual.hh
  AxisVisual.hh
  Camera.hh
  CameraVisual.hh
  COMVisual.hh
  ContactVisual.hh
  Conversions.hh
  CustomPSSMShadowCameraSetup.hh
  DepthCamera.hh
  Distortion.hh
  DynamicLines.hh
  DynamicRenderable.hh
  FPSViewController.hh
  GpuLaser.hh
  GpuLaserDataIterator.hh
  GpuLaserDataIteratorImpl.hh
  Grid.hh
  Heightmap.hh
  InertiaVisual.hh
  JointVisual.hh
  LaserVisual.hh
  LensFlare.hh
  LinkFrameVisual.hh
  LogicalCameraVisual.hh
  SonarVisual.hh
  Light.hh
  Material.hh
  MovableText.hh
  OrbitViewController.hh
  OriginVisual.hh
  OrthoViewController.hh
  Projector.hh
  RayQuery.hh
  RenderEngine.hh
  RenderEvents.hh
  RenderingIface.hh
  RenderTypes.hh
  Road2d.hh
  RFIDVisual.hh
  RFIDTagVisual.hh
  RTShaderSystem.hh
  Scene.hh
  SelectionObj.hh
  TransmitterVisual.hh
  UserCamera.hh
  VideoVisual.hh
  ViewController.hh
  Visual.hh
  WideAngleCamera.hh
  WireBox.hh
  WindowManager.hh
  WrenchVisual.hh
  ogre_gazebo.h
)

# This captures headers that should not be installed.
set (internal_headers
  MarkerManager.hh
  MarkerVisual.hh
)

if (${OGRE_VERSION} VERSION_GREATER 1.7.4)
  add_subdirectory(deferred_shading)
endif()

add_subdirectory(selection_buffer)
add_subdirectory(skyx)

if (HAVE_OCULUS)
  set (sources ${sources}
    OculusCamera.cc
  )
  set (headers ${headers}
    OculusCamera.hh
  )
endif ()

set (gtest_sources
  GpuLaserDataIterator_TEST.cc
  RenderingConversions_TEST.cc
)

gz_build_tests(${gtest_sources} EXTRA_LIBS gazebo_rendering)

set (gtest_sources_dri
  ApplyWrenchVisual_TEST.cc
  ArrowVisual_TEST.cc
  AxisVisual_TEST.cc
  Camera_TEST.cc
  CameraVisual_TEST.cc
  COMVisual_TEST.cc
  ContactVisual_TEST.cc
  Distortion_TEST.cc
  GpuLaser_TEST.cc
  Grid_TEST.cc
  Heightmap_TEST.cc
  InertiaVisual_TEST.cc
  JointVisual_TEST.cc
  LaserVisual_TEST.cc
  LogicalCameraVisual_TEST.cc
  LinkFrameVisual_TEST.cc
  MovableText_TEST.cc
  RenderingMaterial_TEST.cc
  OriginVisual_TEST.cc
  RenderEngine_TEST.cc
  RenderEvents_TEST.cc
  RenderingLight_TEST.cc
  RFIDVisual_TEST.cc
  RTShaderSystem_TEST.cc
  Scene_TEST.cc
  SelectionObj_TEST.cc
  SonarVisual_TEST.cc
  TransmitterVisual_TEST.cc
  Visual_TEST.cc
  WrenchVisual_TEST.cc
)

gz_build_dri_tests(${gtest_sources_dri}
  EXTRA_LIBS gazebo_rendering gazebo_test_fixture)

set (rendering_headers "")
foreach (hdr ${headers})
  set (rendering_headers "${rendering_headers}#include \"gazebo/rendering/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/rendering.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/rendering.hh)

link_directories(
  ${ogre_library_dirs}
)

add_definitions(-DSKYX_LIB)

gz_add_library(gazebo_rendering ${sources})

target_compile_definitions(gazebo_rendering
  PRIVATE BUILDING_DLL_GZ_RENDERING
)

target_link_libraries(gazebo_rendering
  gazebo_common
  gazebo_msgs
  gazebo_transport
  ${ogre_libraries}
  ${OPENGL_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
  ${IGNITION-MSGS_LIBRARIES}
)

if (HAVE_OCULUS)
  target_link_libraries(gazebo_rendering ${OculusVR_LIBRARIES})
endif()

if (NOT APPLE AND NOT WIN32)
  target_link_libraries(gazebo_rendering X11)
endif()

if (USE_PCH)
  add_pch(gazebo_rendering rendering_pch.hh ${Boost_PKGCONFIG_CFLAGS})
endif()

gz_install_library(gazebo_rendering)
gz_install_includes("rendering" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/rendering.hh)
