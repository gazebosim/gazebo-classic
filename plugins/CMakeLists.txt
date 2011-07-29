include_directories( 
  ${PROJECT_BINARY_DIR}/src 
  ${PROJECT_SOURCE_DIR}/src 
)

set (plugins model_push ray_test pr2_pose_test world_edit)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} gazebo_msgs gazebo_common gazebo_transport gazebo_physics gazebo_sensors)
  install (TARGETS ${src} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/plugins/)
endforeach (src ${plugins})
