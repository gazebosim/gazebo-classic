include_directories( 
  ${PROJECT_BINARY_DIR}/src 
  ${PROJECT_SOURCE_DIR}/src 
)

set (plugins camera_plugin)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} gazebo_server)
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/plugins/)
endforeach (src ${plugins})
