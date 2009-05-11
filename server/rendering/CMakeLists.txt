include (${gazebo_cmake_dir}/GazeboUtils.cmake)

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
)

APPEND_TO_SERVER_SOURCES(${sources})
