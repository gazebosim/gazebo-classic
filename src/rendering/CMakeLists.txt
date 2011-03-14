include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories (
  .
  ${PROJECT_SOURCE_DIR}/src/common
  ${PROJECT_SOURCE_DIR}/src/common/transport
  ${CMAKE_BINARY_DIR}/src/msgs
  ${ogre_include_dirs}
  ${XML_INCLUDE_DIRS}
)

set (sources OgreMovableText.cc
             RenderEngine.cc
             OgreDynamicRenderable.cc
             OgreDynamicLines.cc
             Visual.cc
             Camera.cc
             UserCamera.cc
             OgreHeightmap.cc
             Material.cc
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
)

set (headers OgreMovableText.hh
             RenderEngine.hh
             OgreDynamicRenderable.hh
             OgreDynamicLines.hh
             Visual.hh
             Camera.hh
             UserCamera.hh
             OgreHeightmap.hh
             Material.hh
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
)

add_library(gazebo_rendering SHARED ${sources})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_rendering PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

target_link_libraries( gazebo_rendering ${OGRE_LIBRARIES} X11 ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}})
install ( TARGETS gazebo_rendering DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
