include_directories( 
  ${PROJECT_BINARY_DIR}/src 
  ${PROJECT_SOURCE_DIR}/src 
)

if (HAVE_CEGUI)
  set (plugins presentation animation)
endif (HAVE_CEGUI)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} gazebo_gui gazebo_server )
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/plugins/test)
endforeach (src ${plugins})
