cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Qt4)

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

QT4_WRAP_CPP(headers_MOC MainWindowExampleWidget.hh)

add_library(main_window_example_widget SHARED ${headers_MOC}
  MainWindowExampleWidget.cc)
target_link_libraries(main_window_example_widget ${GAZEBO_LIBRARIES} 
    ${OGRE_LIBRARIES} 
    ${OGRE-Terrain_LIBRARIES}
    ${QT_LIBRARIES}
    ${PROTOBUF_LIBRARIES})
