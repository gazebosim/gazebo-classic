file(GLOB files "*.world")

if (BUILD_OSX_BUNDLE)
  install(FILES ${files} DESTINATION ${OSX_BUNDLE_ROOT}/Resources/worlds/)
else()
  install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_MAJOR_VERSION}/worlds/)
endif()
