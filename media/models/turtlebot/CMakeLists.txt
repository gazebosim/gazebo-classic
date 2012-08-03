SET (files 
  68-02403-125_Spacer.dae
  68-02421-8000-RA_Turtlebot_F-F_Standoff_color.png
  68-02421-8000-RA_Turtlebot_F-F_Standoff.dae
  68-04552-1000-RA_Turtlebot_M-F_Standoff_color.png
  68-04552-1000-RA_Turtlebot_M-F_Standoff.dae
  68-04552-2000-RA_Turtlebot_M-F_Standoff_color.png
  68-04552-2000-RA_Turtlebot_M-F_Standoff.dae
  68-04556-RA_Kinect_Standoff_Assy.3ds
  68-04556-RA_Kinect_Standoff_Assy.dae
  CMakeLists.txt
  create_body.dae
  create_body.tga
  kinect.dae
  kinect.tga
  plate_0_logo.dae
  plate_0_logo.tga
  plate_1_logo.dae
  plate_1_logo.tga
  plate_2_logo.dae
  plate_2_logo.tga
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/models/turtlebot)
