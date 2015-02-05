include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROTOBUF_INCLUDE_DIR}
  ${SDFormat_INCLUDE_DIRS}
  ${OGRE_INCLUDE_DIRS}
)

link_directories(
  ${SDFormat_LIBRARY_DIRS}
)

if (HAVE_BULLET)
  link_directories(${BULLET_LIBRARY_DIRS})
endif()

set (plugins
  ExceptionModelPluginConstructor
  ExceptionModelPluginInit
  ExceptionModelPluginLoad
  ForceTorqueModelRemovalTestPlugin
  Issue1208Plugin
  ModelTrajectoryTestPlugin
  SpringTestPlugin
)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo gazebo_sensors ${ogre_libraries})
endforeach (src ${plugins})

set ( qt_plugins
  Issue1434Plugin
)

include (${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

foreach (src ${qt_plugins})
  QT4_WRAP_CPP(headers_MOC ${src}.hh)

  add_library(${src} SHARED ${headers_MOC} ${src}.cc)
  target_link_libraries(${src} ${GAZEBO_LIBRARIES} ${OGRE_LIBRARIES}
    ${QT_LIRARIES})
endforeach (src ${qt_plugins})
