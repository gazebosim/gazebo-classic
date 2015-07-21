if (WIN32)
    add_definitions(-DBUILDING_DLL)
endif()

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
  target_link_libraries(${src} libgazebo ${ogre_libraries})
endforeach (src ${plugins})
