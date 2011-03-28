include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories (
  .
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_BINARY_DIR}/src/msgs
  ${OGRE_INCLUDE_DIRS}
  ${XML_INCLUDE_DIRS}
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
)

add_library(gazebo_rendering SHARED ${sources})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_rendering PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

target_link_libraries( gazebo_rendering gazebo_common 
                                        ${ogre_libraries} 
                                        ${OPENGL_LIBRARIES} 
                                        X11 )

install ( TARGETS gazebo_rendering DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo/rendering COMPONENT headers)
