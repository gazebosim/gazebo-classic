cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package (Qt4 REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Gazebo REQUIRED)

include_directories(
  ${GAZEBO_SOURCE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
  ${OGRE-RTShaderSystem_INCLUDE_DIRS}
)

link_directories(
  ${SDFormat_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${OGRE-RTShaderSystem_LIBRARY_DIRS}
)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

set ( qt_plugins
  Issue1434PluginExample
)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

foreach (src ${qt_plugins})
  QT4_WRAP_CPP(headers_MOC ${src}.hh)

  add_library(${src} SHARED ${headers_MOC} ${src}.cc)
  target_link_libraries(${src} ${GAZEBO_LIBRARIES} ${OGRE_LIBRARIES}
    ${OGRE-RTShaderSystem_LIBRARIES} ${QT_LIRARIES})
endforeach (src ${qt_plugins})
