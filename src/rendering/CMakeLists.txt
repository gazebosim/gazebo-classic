include (${gazebo_cmake_dir}/GazeboUtils.cmake)


include_directories(SYSTEM
  ${OGRE_INCLUDE_DIRS}
  ${CEGUI_include_dirs}
  ${CEGUI_INCLUDE_DIRS}
  ${CEGUI_OGRE_include_dirs}
  ${CEGUI_OGRE_INCLUDE_DIRS}
)

set (sources MovableText.cc
             RenderEngine.cc
             RenderEvents.cc
             DynamicRenderable.cc
             DynamicLines.cc
             Visual.cc
             Camera.cc
             DepthCamera.cc
             UserCamera.cc
             Heightmap.cc
             Light.cc
             RTShaderSystem.cc
             SelectionObj.cc
             ViewController.cc
             FPSViewController.cc
             OrbitViewController.cc
             Grid.cc
             Scene.cc
             WindowManager.cc
             Conversions.cc
             Rendering.cc
             Material.cc
             GUIOverlay.cc
             LaserVisual.cc
             CameraVisual.cc
             ContactVisual.cc
)

set (headers MovableText.hh
             RenderEngine.hh
             RenderEvents.hh
             Visual.hh
             Camera.hh
             DepthCamera.hh
             UserCamera.hh
             Light.hh
             SelectionObj.hh
             ViewController.hh
             FPSViewController.hh
             OrbitViewController.hh
             Scene.hh
             WindowManager.hh
             RenderTypes.hh
             Rendering.hh
             Material.hh
             GUIOverlay.hh
             LaserVisual.hh
             CameraVisual.hh
             ContactVisual.hh
             ogre.h
             cegui.h
)

set (rendering_headers "" CACHE INTERNAL "rendering headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(rendering_headers "Rendering Headers" "#include \"rendering/${hdr}\"\n")
endforeach()
configure_file ( ${CMAKE_CURRENT_SOURCE_DIR}/rendering.h.in ${CMAKE_CURRENT_BINARY_DIR}/rendering.h )

add_library(gazebo_rendering SHARED ${sources})

target_link_libraries( gazebo_rendering gazebo_common 
                                        gazebo_sdf_interface
                                        ${ogre_libraries} 
                                        ${OPENGL_LIBRARIES} 
                                        ${CEGUI_LIBRARIES}
                                        ${CEGUI_OGRE_LIBRARIES}
                                        X11 )
INSTALL_LIBRARY(gazebo_rendering)
INSTALL_INCLUDES("rendering" ${headers})
