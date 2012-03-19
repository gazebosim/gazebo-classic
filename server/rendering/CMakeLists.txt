include (${gazebo_cmake_dir}/GazeboUtils.cmake)

add_definitions(${QT_DEFINITIONS})
include (${QT_USE_FILE})

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
             ViewController.cc
             FPSViewController.cc
             OrbitViewController.cc
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
             ViewController.hh
             FPSViewController.hh
             OrbitViewController.hh
)

add_library(gazebo_rendering SHARED ${sources})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_rendering PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

target_link_libraries( gazebo_rendering ${boost_libraries} ${ogre_libraries} ${OPENGL_LIBRARIES} ${FLTK_LIBRARIES} ${QT_LIBRARIES} xml2 X11)
install ( TARGETS gazebo_rendering DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
APPEND_TO_SERVER_HEADERS(${headers})
