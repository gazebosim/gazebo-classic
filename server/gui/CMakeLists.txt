include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories( ${FLTK_INCLUDE_DIR} )

LINK_DIRECTORIES(  
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${CMAKE_BINARY_DIR}/server/gui 
 ${boost_library_dirs} 
 ${freeimage_library_dir}
 ${gazeboserver_link_dirs} 
)

set (sources Gui.cc
             GLWindow.cc
             MainMenu.cc
             Toolbar.cc
             Sidebar.cc
             StatusBar.cc
             GLFrameManager.cc
             GLFrame.cc
             Events.cc
             EntityMaker.cc
             BoxMaker.cc
             SphereMaker.cc
             CylinderMaker.cc
             PointLightMaker.cc
             SpotLightMaker.cc
             DirectionalLightMaker.cc
             ParamBrowser.cc
)

set (headers Gui.hh
             GLWindow.hh
             MainMenu.hh
             Toolbar.hh
             Sidebar.hh
             StatusBar.hh
             GLFrameManager.hh
             GLFrame.hh
             Events.hh
             EntityMaker.hh
             BoxMaker.hh
             SphereMaker.hh
             CylinderMaker.hh
             MouseEvent.hh
             PointLightMaker.hh
             SpotLightMaker.hh
             DirectionalLightMaker.hh
             ParamBrowser.hh
)

LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")

add_library(gazebo_gui SHARED ${sources})

target_link_libraries(gazebo_gui ${boost_libraries} ${FLTK_LIBRARIES} Xpm)

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_gui PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

set_target_properties(gazebo_gui PROPERTIES OUTPUT_NAME "gazebo_gui" VERSION ${GAZEBO_VERSION})

install (TARGETS gazebo_gui DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
