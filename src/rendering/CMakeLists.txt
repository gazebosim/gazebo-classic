include (${gazebo_cmake_dir}/GazeboUtils.cmake)


include_directories(SYSTEM
  ${OGRE_INCLUDE_DIRS}
  ${CEGUI_INCLUDE_DIRS}
)

set (sources
  ArrowVisual.cc
  AxisVisual.cc
  Camera.cc
  CameraVisual.cc
  COMVisual.cc
  ContactVisual.cc
  Conversions.cc
  DepthCamera.cc
  DynamicLines.cc
  DynamicRenderable.cc
  FPSViewController.cc
  GpuLaser.cc
  Grid.cc
  GUIOverlay.cc
  Heightmap.cc
  JointVisual.cc
  LaserVisual.cc
  Light.cc
  Material.cc
  MovableText.cc
  OrbitViewController.cc
  Projector.cc
  RenderEngine.cc
  RenderEvents.cc
  Rendering.cc
  RFIDVisual.cc
  RFIDTagVisual.cc
  RTShaderSystem.cc
  Scene.cc
  SelectionObj.cc
  UserCamera.cc
  ViewController.cc
  Visual.cc
  WindowManager.cc
)

set (headers
  ArrowVisual.hh
  AxisVisual.hh
  Camera.hh
  CameraVisual.hh
  COMVisual.hh
  ContactVisual.hh
  Conversions.hh
  DepthCamera.hh
  DynamicLines.hh
  DynamicRenderable.hh
  FPSViewController.hh
  GpuLaser.hh
  Grid.hh
  GUIOverlay.hh
  Heightmap.hh
  JointVisual.hh
  LaserVisual.hh
  Light.hh
  Material.hh
  MovableText.hh
  OrbitViewController.hh
  Projector.hh
  RenderEngine.hh
  RenderEvents.hh
  Rendering.hh
  RFIDVisual.hh
  RFIDTagVisual.hh
  RTShaderSystem.hh
  Scene.hh
  SelectionObj.hh
  UserCamera.hh
  ViewController.hh
  Visual.hh
  WindowManager.hh
  ogre.h
  cegui.h)

set (rendering_headers "" CACHE INTERNAL "rendering headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(rendering_headers "Rendering Headers" "#include \"rendering/${hdr}\"\n")
endforeach()
configure_file ( ${CMAKE_CURRENT_SOURCE_DIR}/rendering.h.in ${CMAKE_CURRENT_BINARY_DIR}/rendering.h )

gz_add_library(gazebo_rendering ${sources})

target_link_libraries( gazebo_rendering gazebo_common 
                                        gazebo_sdf_interface
                                        ${ogre_libraries} 
                                        ${OPENGL_LIBRARIES} 
                                        ${CEGUI_LIBRARIES}
                                        X11 )
                                      

INSTALL_LIBRARY(gazebo_rendering)
INSTALL_INCLUDES("rendering" ${headers})
