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
link_directories(${tinyxml_LIBRARY_DIRS})

set (sources
  ApplyWrenchVisual.cc
  ArrowVisual.cc
  AxisVisual.cc
  Camera.cc
  CameraVisual.cc
  COMVisual.cc
  ContactVisual.cc
  Conversions.cc
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
  LinkFrameVisual.cc
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
  DepthCamera.hh
  Distortion.hh
  DynamicLines.hh
  DynamicRenderable.hh
  FPSViewController.hh
  GpuLaser.hh
  Grid.hh
  Heightmap.hh
  InertiaVisual.hh
  JointVisual.hh
  LaserVisual.hh
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
  WireBox.hh
  WindowManager.hh
  WrenchVisual.hh
  ogre_gazebo.h
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

set (gtest_sources)

gz_build_tests(${gtest_sources})

set (gtest_sources_dri
  COMVisual_TEST.cc
  GpuLaser_TEST.cc
  Heightmap_TEST.cc
  JointVisual_TEST.cc
  LinkFrameVisual_TEST.cc
  OriginVisual_TEST.cc
  Scene_TEST.cc
  SelectionObj_TEST.cc
  SonarVisual_TEST.cc
  TransmitterVisual_TEST.cc
  Visual_TEST.cc
)

gz_build_dri_tests(${gtest_sources_dri})

set (rendering_headers "" CACHE INTERNAL "rendering headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(rendering_headers
    "Rendering Headers" "#include \"gazebo/rendering/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/rendering.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/rendering.hh)

link_directories(
  ${ogre_library_dirs}
)

set (CMAKE_CXX_FLAGS
  "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_GZ_RENDERING -DSKYX_LIB")
if (WIN32)
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /FIWinsock2.h")
endif()

gz_add_library(gazebo_rendering ${sources})

target_link_libraries(gazebo_rendering
  gazebo_common
  gazebo_msgs
  gazebo_transport
  ${ogre_libraries}
  ${OPENGL_LIBRARIES}
  ${tinyxml_LIBRARIES}
)

if (HAVE_OCULUS)
  target_link_libraries(gazebo_rendering ${OculusVR_LIBRARIES})
endif()

if (NOT APPLE AND NOT WIN32)
  target_link_libraries(gazebo_rendering X11)
endif()

gz_install_library(gazebo_rendering)
gz_install_includes("rendering" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/rendering.hh)
