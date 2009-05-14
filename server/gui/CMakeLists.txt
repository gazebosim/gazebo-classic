SET (sources Gui.cc
             GLWindow.cc
             MainMenu.cc
             Toolbar.cc
             StatusBar.cc
             GLFrameManager.cc
             GLFrame.cc
)

SET (headers Gui.hh
             GLWindow.hh
             MainMenu.hh
             Toolbar.hh
             StatusBar.hh
             GLFrameManager.hh
             GLFrame.hh
)

ADD_LIBRARY(gazebo_gui STATIC ${sources})
ADD_LIBRARY(gazebo_gui-shared SHARED ${sources})

#SET_TARGET_PROPERTIES(gazebo_gui-static PROPERTIES OUTPUT_NAME "gazebo_gui")
SET_TARGET_PROPERTIES(gazebo_gui-shared PROPERTIES OUTPUT_NAME "gazebo_gui")

INSTALL (TARGETS gazebo_gui gazebo_gui-shared DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
