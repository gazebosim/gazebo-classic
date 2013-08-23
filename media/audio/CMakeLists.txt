SET(files cheer.wav cheer.ogg cheer.mp3)
INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/audio)
