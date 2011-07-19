include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories (
  .
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_BINARY_DIR}/src/msgs
  ${OGRE_INCLUDE_DIRS}
)

set (sources MovableText.cc
             RenderEngine.cc
             DynamicRenderable.cc
             DynamicLines.cc
             Visual.cc
             Camera.cc
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
             RenderState.cc
             WindowManager.cc
             Conversions.cc
             Rendering.cc
             Material.cc
)

set (headers MovableText.hh
             RenderEngine.hh
             DynamicRenderable.hh
             DynamicLines.hh
             Visual.hh
             Camera.hh
             UserCamera.hh
             Heightmap.hh
             Light.hh
             RTShaderSystem.hh
             SelectionObj.hh
             ViewController.hh
             FPSViewController.hh
             OrbitViewController.hh
             Grid.hh
             Scene.hh
             RenderState.hh
             WindowManager.hh
             RenderTypes.hh
             Conversions.hh
             Rendering.hh
             Material.hh
)

add_library(gazebo_rendering SHARED ${sources})

target_link_libraries( gazebo_rendering gazebo_common 
                                        gazebo_sdf_interface
                                        gazebo_sdf_parser
                                        ${ogre_libraries} 
                                        ${OPENGL_LIBRARIES} 
                                        X11 )
INSTALL_LIBRARY(gazebo_rendering)
INSTALL_INCLUDES("rendering" ${headers})
