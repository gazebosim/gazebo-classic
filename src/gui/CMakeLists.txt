include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

link_directories(  
  ${BOOST_LIBRARY_DIRS} 
)

add_definitions( ${QT_DEFINITIONS} )

add_subdirectory(qtpropertybrowser)

set (sources main.cc 
             Gui.cc 
             MainWindow.cc 
             RenderWidget.cc 
             GLWidget.cc 
             TimePanel.cc
             InsertModelWidget.cc
             ModelListWidget.cc
             WorldPropertiesWidget.cc
             EntityMaker.cc
             BoxMaker.cc
             SphereMaker.cc
             CylinderMaker.cc
             LightMaker.cc
             GuiEvents.cc
             ModelEditWidget.cc
             MeshMaker.cc
             ModelMaker.cc
             ModelRightMenu.cc
)
             #             JointControlWidget.cc

set (qt_headers MainWindow.hh 
                RenderWidget.hh 
                GLWidget.hh 
                TimePanel.hh
                InsertModelWidget.hh
                ModelListWidget.hh
                WorldPropertiesWidget.hh
                ModelEditWidget.hh
                ModelRightMenu.hh
)
                #JointControlWidget.hh

set (headers Gui.hh
             EntityMaker.hh
             BoxMaker.hh
             SphereMaker.hh
             CylinderMaker.hh
             MeshMaker.hh
             ModelMaker.hh
             GuiEvents.hh
             LightMaker.hh
             qt.h
)

set (resources images.qrc)

QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

gz_add_executable(gzclient ${sources} ${headers_MOC} ${headers} ${resources_RCC})
gz_add_library(gazebo_gui ${sources} ${headers_MOC} ${headers} ${resources_RCC})

target_link_libraries(gazebo_gui gazebo_common
                                 gazebo_transport
                                 gazebo_sdf_interface
                                 gazebo_rendering
                                 gazebo_sensors
                                 gazebo_msgs
                                 gzqtpropertybrowser 
                                 ${QT_LIBRARIES} 
                                 ${ogre_libraries} 
                                 ${boost_libraries} 
                                 pthread)

target_link_libraries(gzclient  gazebo_common
                                gazebo_transport
                                gazebo_sdf_interface
                                gazebo_rendering
                                gazebo_sensors
                                gazebo_msgs
                                gzqtpropertybrowser 
                                ${QT_LIBRARIES} 
                                ${ogre_libraries} 
                                ${boost_libraries} 
                                pthread)

 

add_definitions(${QT_DEFINITIONS})

INSTALL_LIBRARY(gazebo_gui)
INSTALL_INCLUDES("gui" ${headers})
INSTALL_EXECUTABLE(gzclient)
