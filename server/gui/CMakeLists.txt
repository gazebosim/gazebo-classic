INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

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

LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")
SET_SOURCE_FILES_PROPERTIES(${sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")

#ADD_LIBRARY(gazebo_gui STATIC ${sources})
ADD_LIBRARY(gazebo_gui-shared SHARED ${sources})

#TARGET_LINK_LIBRARIES(gazebo_gui ${FLTK_LIBRARIES} )
TARGET_LINK_LIBRARIES(gazebo_gui-shared ${FLTK_LIBRARIES} )

#SET_TARGET_PROPERTIES(gazebo_gui PROPERTIES OUTPUT_NAME "gazebo_gui")
SET_TARGET_PROPERTIES(gazebo_gui-shared PROPERTIES OUTPUT_NAME "gazebo_gui" VERSION ${GAZEBO_VERSION})

INSTALL (TARGETS gazebo_gui-shared DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
