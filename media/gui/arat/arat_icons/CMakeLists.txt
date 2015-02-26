set (files 
      grasp_1.jpg
      grasp_3.jpg
      grasp_5.jpg
      grip_1.jpg
      grip_3.jpg
      pinch_1.jpg
      grasp_2.jpg
      grasp_4.jpg
      grasp_6.jpg
      grip_2.jpg
      grip_4.jpg
      pinch_2.jpg
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/gui/arat/arat_icons)

