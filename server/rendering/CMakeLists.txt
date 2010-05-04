include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories (
  .
  ${OGRE_INCLUDE_DIRS}
)

set (sources OgreMovableText.cc
             OgreCreator.cc
             OgreAdaptor.cc
             OgreFrameListener.cc
             OgreDynamicRenderable.cc
             OgreDynamicLines.cc
             OgreHUD.cc
             OgreVisual.cc
             OgreCamera.cc
             CameraManager.cc
             UserCamera.cc
             Image.cc
             OgreHeightmap.cc
             Color.cc
             Material.cc
             Light.cc
             RTShaderSystem.cc
             SelectionObj.cc
)

set (headers OgreMovableText.hh
             OgreCreator.hh
             OgreAdaptor.hh
             OgreFrameListener.hh
             OgreDynamicRenderable.hh
             OgreDynamicLines.hh
             OgreHUD.hh
             OgreVisual.hh
             OgreCamera.hh
             CameraManager.hh
             UserCamera.hh
             Image.hh
             OgreHeightmap.hh
             Color.hh
             Material.hh
             Light.hh
             RTShaderSystem.hh
             SelectionObj.hh
)

add_library(gazebo_rendering SHARED ${sources})
target_link_libraries( gazebo_rendering ${OGRE_LIBRARIES} X11)
install ( TARGETS gazebo_rendering DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
APPEND_TO_SERVER_HEADERS(${headers})
