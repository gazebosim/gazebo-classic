INCLUDE_DIRECTORIES( 
  ${CMAKE_SOURCE_DIR}/libgazebo 
  ${CMAKE_SOURCE_DIR}/server 
  ${CMAKE_SOURCE_DIR}/server/rendering 
  ${CMAKE_SOURCE_DIR}/server/physics 
  ${gazeboserver_include_dirs}
)

SET (sources Gui.cc
             GLWindow.cc
             MainMenu.cc
             Toolbar.cc
             StatusBar.cc
             GLFrameManager.cc
             GLFrame.cc
)

ADD_LIBRARY(gazebo_gui STATIC ${sources})
