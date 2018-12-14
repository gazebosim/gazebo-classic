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
  ${CURL_LIBDIR}
  ${IGNITION-MATH_LIBRARY_DIRS}
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
  PluginInterfaceTest
  SpringTestPlugin
)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} libgazebo ${ogre_libraries})
  add_dependencies(tests ${src})
endforeach (src ${plugins})

target_link_libraries(PluginInterfaceTest gtest gtest_main)