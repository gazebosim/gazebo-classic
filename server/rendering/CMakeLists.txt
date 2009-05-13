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

ADD_LIBRARY(gazebo_rendering ${sources})
