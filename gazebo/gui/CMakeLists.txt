include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

link_directories(  
  ${Boost_LIBRARY_DIRS} 
)

add_definitions(${QT_DEFINITIONS})

add_subdirectory(qtpropertybrowser)

set (sources
  main.cc 
  ../gazebo.cc
  Actions.cc
  Gui.cc 
  MainWindow.cc 
  RenderWidget.cc 
  GLWidget.cc 
  TimePanel.cc
  InsertModelWidget.cc
  ModelListWidget.cc
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
  JointControlWidget.cc
  ToolsWidget.cc
  TopicSelector.cc
  viewers/ImageView.cc
  viewers/TopicView.cc
  viewers/TextView.cc
  viewers/ViewFactory.cc
)

set (qt_headers
  Actions.hh
  MainWindow.hh 
  RenderWidget.hh 
  GLWidget.hh 
  TimePanel.hh
  InsertModelWidget.hh
  ModelListWidget.hh
  ModelEditWidget.hh
  ModelRightMenu.hh
  JointControlWidget.hh
  ToolsWidget.hh
  TopicSelector.hh
  viewers/ImageView.hh
  viewers/TextView.hh
  viewers/TopicView.hh
)

set (headers
  Gui.hh
  EntityMaker.hh
  BoxMaker.hh
  SphereMaker.hh
  CylinderMaker.hh
  MeshMaker.hh
  ModelMaker.hh
  GuiEvents.hh
  LightMaker.hh
  qt.h
  viewers/ViewFactory.hh
  )

set (resources resources.qrc)

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
                                 ${Boost_LIBRARIES} 
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
                                ${Boost_LIBRARIES} 
                                pthread)

 

add_definitions(${QT_DEFINITIONS})

gz_install_library(gazebo_gui)
gz_install_includes("gui" ${headers})
gz_install_executable(gzclient)
