include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

link_directories(  
  ${BOOST_LIBRARY_DIRS} 
)

add_definitions( ${QT_DEFINITIONS} )

set (sources main.cc 
             Gui.cc 
             MainWindow.cc 
             RenderWidget.cc 
             GLWidget.cc 
             TimePanel.cc
             InsertModelWidget.cc
             EntityMaker.cc
             BoxMaker.cc
             SphereMaker.cc
             CylinderMaker.cc
             )

set (qt_headers 
             MainWindow.hh 
             RenderWidget.hh 
             GLWidget.hh 
             TimePanel.hh
             InsertModelWidget.hh
             )

set (headers Gui.hh
             EntityMaker.hh
             BoxMaker.hh
             SphereMaker.hh
             CylinderMaker.hh
             )


set (resources images.qrc)

QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

#add_library(gazebo_gui SHARED ${sources} ${headers_MOC} ${headers} ${resources_RCC})
                                         
#target_link_libraries(gazebo_gui Xpm ${QT_LIBRARIES} ${ogre_libraries} ${boost_libraries} gazebo_physics gazebo_msgs)

#set_target_properties(gazebo_gui PROPERTIES OUTPUT_NAME "gazebo_gui" VERSION ${GAZEBO_VERSION})
#INSTALL_LIBRARY(gazebo_gui)
#INSTALL_INCLUDES( "gui" ${headers} ${qt_headers})

add_executable(gzclient ${sources} ${headers_MOC} ${headers} ${resources_RCC})

target_link_libraries( gzclient gazebo_common
                                gazebo_transport
                                gazebo_sdf_interface
                                gazebo_sdf_parser
                                gazebo_rendering
                                gazebo_msgs
                                Xpm 
                                ${QT_LIBRARIES} 
                                ${ogre_libraries} 
                                ${boost_libraries} 
                                pthread)
INSTALL_EXECUTABLE( gzclient )
