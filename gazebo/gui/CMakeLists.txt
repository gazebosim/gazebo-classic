include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

link_directories(
  ${Boost_LIBRARY_DIRS}
)

add_definitions(${QT_DEFINITIONS})

add_subdirectory(qtpropertybrowser)
add_subdirectory(model_editor)

set (sources
  Actions.cc
  BoxMaker.cc
  BuildingEditor.cc
  CylinderMaker.cc
  DataLogger.cc
  Editor.cc
  EntityMaker.cc
  RenderWidget.cc
  GLWidget.cc
  Gui.cc
  GuiEvents.cc
  InsertModelWidget.cc
  JointControlWidget.cc
  LightMaker.cc
  MainWindow.cc
  MeshMaker.cc
  ModelEditWidget.cc
  ModelListWidget.cc
  ModelMaker.cc
  ModelRightMenu.cc
  MouseEventHandler.cc
  SphereMaker.cc
  TerrainEditor.cc
  TerrainEditorPalette.cc
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
  BuildingEditor.hh
  DataLogger.hh
  Editor.hh
  GLWidget.hh
  InsertModelWidget.hh
  JointControlWidget.hh
  MainWindow.hh
  ModelListWidget.hh
  ModelEditWidget.hh
  ModelRightMenu.hh
  RenderWidget.hh
  TerrainEditor.hh
  TerrainEditorPalette.hh
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
  MouseEventHandler.hh
  viewers/ViewFactory.hh
)

set (qt_tests 
  DataLogger_TEST.cc
  TimePanel_TEST.cc
  viewers/ImagesView_TEST.cc
)

gz_build_qt_tests(${qt_tests})

set (resources resources.qrc)


QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

gz_add_executable(gzclient ${sources} main.cc ${headers_MOC} ${headers} ${resources_RCC})
gz_add_library(gazebo_gui ${sources} ${headers_MOC} ${headers} ${resources_RCC})

add_dependencies(gazebo_gui gazebo_msgs)
add_dependencies(gzclient gazebo_msgs)

target_link_libraries(gazebo_gui gazebo_common
                                 gazebo_transport
                                 gazebo_sdf_interface
                                 gazebo_rendering
                                 gazebo_sensors
                                 gazebo_msgs
                                 gazebo_model_editor
                                 gzqtpropertybrowser
                                 libgazebo
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
                                libgazebo
                                ${QT_LIBRARIES}
                                ${ogre_libraries}
                                ${Boost_LIBRARIES}
                                pthread)

add_definitions(${QT_DEFINITIONS})

gz_install_library(gazebo_gui)
gz_install_includes("gui" ${headers})
gz_install_executable(gzclient)
