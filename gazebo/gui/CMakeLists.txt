include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

link_directories(
  ${Boost_LIBRARY_DIRS}
)

add_definitions(${QT_DEFINITIONS})

add_subdirectory(qtpropertybrowser)
add_subdirectory(qwt)
add_subdirectory(model_editor)

set (sources
  ../gazebo.cc
  Actions.cc
  BoxMaker.cc
  CylinderMaker.cc
  Diagnostics.cc
  EntityMaker.cc
  RenderWidget.cc
  GLWidget.cc
  Gui.cc
  GuiEvents.cc
  IncrementalPlot.cc
  InsertModelWidget.cc
  JointControlWidget.cc
  LightMaker.cc
  main.cc
  MainWindow.cc
  MeshMaker.cc
  ModelEditWidget.cc
  ModelListWidget.cc
  ModelMaker.cc
  ModelRightMenu.cc
  SphereMaker.cc
  TimePanel.cc
  ToolsWidget.cc
  TopicSelector.cc
  viewers/ImageView.cc
  viewers/ImagesView.cc
  viewers/LaserView.cc
  viewers/TopicView.cc
  viewers/TextView.cc
  viewers/ViewFactory.cc
)

set (qt_headers
  Actions.hh
  Diagnostics.hh
  GLWidget.hh
  IncrementalPlot.hh
  InsertModelWidget.hh
  JointControlWidget.hh
  MainWindow.hh
  ModelListWidget.hh
  ModelEditWidget.hh
  ModelRightMenu.hh
  RenderWidget.hh
  TimePanel.hh
  ToolsWidget.hh
  TopicSelector.hh
  viewers/ImageView.hh
  viewers/ImagesView.hh
  viewers/LaserView.hh
  viewers/TextView.hh
  viewers/TopicView.hh
)

set (headers
  qt.h
  qt_test.h
  BoxMaker.hh
  CylinderMaker.hh
  EntityMaker.hh
  Gui.hh
  GuiEvents.hh
  LightMaker.hh
  MeshMaker.hh
  ModelMaker.hh
  SphereMaker.hh
  viewers/ViewFactory.hh
)

set (display_tests 
  TimePanel_TEST.cc
  viewers/ImagesView_TEST.cc
)

gz_build_display_tests(${display_tests})

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
                                 gazebo_model_editor
                                 gzqtpropertybrowser
                                 gzqwt
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
                                gazebo_model_editor
                                gzqtpropertybrowser
                                gzqwt
                                ${QT_LIBRARIES}
                                ${ogre_libraries}
                                ${Boost_LIBRARIES}
                                pthread)



add_definitions(${QT_DEFINITIONS})

gz_install_library(gazebo_gui)
gz_install_includes("gui" ${headers})
gz_install_executable(gzclient)
