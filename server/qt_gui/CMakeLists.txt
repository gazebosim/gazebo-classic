include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (${QT_USE_FILE})

include_directories(
  ${CMAKE_BINARY_DIR}/server/qt_gui
  )

link_directories(${BOOST_LIBRARY_DIRS}) 
add_definitions(${QT_DEFINITIONS})

set (sources Gui.cc GLWindow.cc)

set (qt_headers Gui.hh GLWindow.hh)

set (headers qt.h MouseEvent.hh)

set (ui_files sharon_simple.ui)
QT4_WRAP_UI(UI_SRCS ${ui_files})
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

add_library(gazebo_gui SHARED ${sources} ${UI_SRCS} ${headers_MOC} ${headers} ${resources_RCC})

target_link_libraries(gazebo_gui ${QT_LIBRARIES} 
                                 ${ogre_libraries} 
                                 ${boost_libraries} 
                                 pthread)

install(TARGETS gazebo_gui DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
