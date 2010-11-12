include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories (
  .
  ${OGRE_INCLUDE_DIRS}
)

set (sources OgreMovableText.cc
             OgreCreator.cc
             OgreAdaptor.cc
             OgreDynamicRenderable.cc
             OgreDynamicLines.cc
             OgreHUD.cc
             OgreVisual.cc
             Camera.cc
             UserCamera.cc
             Image.cc
             OgreHeightmap.cc
             Color.cc
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
)

set (headers OgreMovableText.hh
             OgreCreator.hh
             OgreAdaptor.hh
             OgreDynamicRenderable.hh
             OgreDynamicLines.hh
             OgreHUD.hh
             OgreVisual.hh
             Camera.hh
             UserCamera.hh
             Image.hh
             OgreHeightmap.hh
             Color.hh
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
APPEND_TO_SERVER_HEADERS(${headers})
