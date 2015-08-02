cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Qt4 REQUIRED)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Terrain OGRE-Terrain)
endif()

find_package(Protobuf REQUIRED)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

include_directories(SYSTEM ${GAZEBO_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}
  ${OGRE-Terrain_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS}
  ${OGRE-Terrain_LIBRARY_DIRS})

QT4_WRAP_CPP(headers_MOC CameraLensControlExample.hh ImageFrame.hh)

add_definitions(-std=c++11)

#message("Found qt libraies " ${QT_LIBRARIES})
#message("Found gazebo libraries " ${GAZEBO_LIBRARIES})

add_library(camera_lens_control_example SHARED ${headers_MOC}
		CameraLensControlExample.cc ImageFrame.cc)
target_link_libraries(camera_lens_control_example ${GAZEBO_LIBRARIES}
    ${OGRE_LIBRARIES} 
    ${OGRE-Terrain_LIBRARIES}
    ${QT_LIBRARIES} Qt4::QtUiTools Qt4::QtXml
    ${PROTOBUF_LIBRARIES})
