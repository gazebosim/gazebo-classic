include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM
  ${OGRE_INCLUDE_DIRS}
  skyx
  skyx/include
)

if (${OGRE_VERSION} VERSION_GREATER 1.7.4)
  add_subdirectory(deferred_shading)
endif()

add_subdirectory(skyx)
add_subdirectory(selection_buffer)

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
  JointVisual.cc
  LaserVisual.cc
  SonarVisual.cc
  Light.cc
  Material.cc
  MovableText.cc
  OrbitViewController.cc
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
  JointVisual.hh
  LaserVisual.hh
  SonarVisual.hh
  Light.hh
  Material.hh
  MovableText.hh
  OrbitViewController.hh
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

if (HAVE_OCULUS)
  set (sources ${sources}
    OculusCamera.cc
  )
  set (headers ${headers}
    OculusCamera.hh
  )
endif ()

set (gtest_sources
)

gz_build_tests(${gtest_sources})

set (gtest_sources_dri
  GpuLaser_TEST.cc
  Heightmap_TEST.cc
  JointVisual_TEST.cc
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

gz_add_library(gazebo_rendering ${sources})

target_link_libraries(gazebo_rendering
  gazebo_common
  gazebo_msgs
  gazebo_skyx
  gazebo_selection_buffer
  gazebo_transport
  ${ogre_libraries}
  ${OPENGL_LIBRARIES}
)

if (HAVE_OCULUS)
  target_link_libraries(gazebo_rendering
    OculusVR
  )
endif()

if (NOT APPLE)
  target_link_libraries(gazebo_rendering X11)
endif()

if (${OGRE_VERSION} VERSION_GREATER 1.7.4)
  target_link_libraries(gazebo_rendering gazebo_rendering_deferred)
endif()

gz_install_library(gazebo_rendering)
gz_install_includes("rendering" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/rendering.hh)
