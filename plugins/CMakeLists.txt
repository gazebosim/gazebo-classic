include_directories( 
  ${PROJECT_BINARY_DIR}/src 
  ${PROJECT_SOURCE_DIR}/src 
)

set (plugins CameraPlugin
             DepthCameraPlugin
             ContactPlugin
             RayPlugin
             Pioneer2dxPlugin)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} gazebo_server)
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo-${GAZEBO_VERSION}/plugins/)
  INSTALL_INCLUDES("plugins" ${src}.hh)
endforeach (src ${plugins})
