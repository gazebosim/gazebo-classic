include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources OgreMovableText.cc
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
)

SET (headers OgreMovableText.hh
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
)

#ADD_LIBRARY(gazebo_rendering ${sources})
APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
