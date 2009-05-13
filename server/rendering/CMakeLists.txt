include (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/server
  ${CMAKE_SOURCE_DIR}/server/gui
  ${gazeboserver_include_dirs} 
  ${boost_include_dirs}
)

SET (sources OgreMovableText.cc
             OgreCreator.cc
             OgreAdaptor.cc
             OgreFrameListener.cc
             OgreDynamicRenderable.cc
             OgreDynamicLines.cc
             OgreSimpleShape.cc
             OgreHUD.cc
             OgreVisual.cc
             OgreCamera.cc
             CameraManager.cc
             UserCamera.cc
             Image.cc
             Color.cc
             OgreHeightmap.cc
)

SET (headers OgreMovableText.hh
             OgreCreator.hh
             OgreAdaptor.hh
             OgreFrameListener.hh
             OgreDynamicRenderable.hh
             OgreDynamicLines.hh
             OgreSimpleShape.hh
             OgreHUD.hh
             OgreVisual.hh
             OgreCamera.hh
             CameraManager.hh
             UserCamera.hh
             Image.hh
             Color.hh
             OgreHeightmap.hh
)

ADD_LIBRARY(gazebo_rendering ${sources})
APPEND_TO_SERVER_HEADERS(${headers})
