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

#APPEND_TO_SERVER_SOURCES(${sources})
#APPEND_TO_SERVER_HEADERS(${headers})
