cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Qt4)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
  pkg_check_modules(OGRE OGRE)
endif()

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS} ${SDFormat_LIBRARY_DIRS} ${OGRE_LIBRARY_DIRS})

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

set ( qt_plugins
  GUIExampleSpawnWidget
)

foreach (src ${qt_plugins})
  QT4_WRAP_CPP(headers_MOC ${src}.hh)

  add_library(${src} SHARED ${headers_MOC} ${src}.cc)
  target_link_libraries(${src} ${GAZEBO_LIBRARIES} ${OGRE_LIBRARIES}
    ${QT_LIRARIES})
endforeach (src ${qt_plugins})
