include_directories( 
  ${PROJECT_BINARY_DIR}/src 
  ${PROJECT_BINARY_DIR}/src/common 
  ${PROJECT_SOURCE_DIR}/src 
)

set (plugins box_push)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} gazebo_msgs gazebo_common gazebo_transport gazebo_physics gazebo_sensors ${ogre_ldflags} ${ODE_LIBRARIES} ${ODE_LDFLAGS})
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/plugins/)
endforeach (src ${plugins})
