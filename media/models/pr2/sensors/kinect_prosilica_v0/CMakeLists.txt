set (files
  115x100_swept_back--coarse.STL
  68-04546_Kinect_Sensor--coarse.STL
  Prosilica_w_Lens--coarse.STL)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/pr2/sensors/kinect_prosilica_v0)
