include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories( ${FLTK_INCLUDE_DIR} )

LINK_DIRECTORIES(  
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${CMAKE_BINARY_DIR}/server/gui 
 ${CMAKE_BINARY_DIR}/server/physics/bullet 
 ${boost_library_dirs} 
 ${bullet_dynamics_dirs} 
 ${bullet_collision_dirs} 
 ${bullet_softbody_dirs} 
 ${bullet_math_dirs} 
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
)

set (headers Gui.hh
             GLWindow.hh
             MainMenu.hh
             Toolbar.hh
             Sidebar.hh
             StatusBar.hh
             GLFrameManager.hh
             GLFrame.hh
)

LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")

add_library(gazebo_gui SHARED ${sources})

#target_link_libraries(gazebo_gui ${FLTK_LIBRARIES} BulletDynamics BulletCollision LinearMath)

target_link_libraries(gazebo_gui ${FLTK_LIBRARIES} Xpm)

set_target_properties(gazebo_gui PROPERTIES OUTPUT_NAME "gazebo_gui" VERSION ${GAZEBO_VERSION})

install (TARGETS gazebo_gui DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
