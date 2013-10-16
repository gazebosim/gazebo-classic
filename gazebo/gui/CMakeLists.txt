include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include ( ${QT_USE_FILE} )

link_directories(
  ${Boost_LIBRARY_DIRS}
)

include_directories(
  ${CMAKE_SOURCE_DIR}/deps
)

add_definitions(${QT_DEFINITIONS})

add_subdirectory(qtpropertybrowser)
add_subdirectory(building)
add_subdirectory(terrain)
add_subdirectory(viewers)

set (sources
  Actions.cc
  BoxMaker.cc
  CylinderMaker.cc
  DataLogger.cc
  Editor.cc
  EntityMaker.cc
  RenderWidget.cc
  GLWidget.cc
  GuiIface.cc
  GuiEvents.cc
  InsertModelWidget.cc
  JointControlWidget.cc
  KeyEventHandler.cc
  LightMaker.cc
  MainWindow.cc
  ModelManipulator.cc
  MeshMaker.cc
  ModelEditWidget.cc
  ModelListWidget.cc
  ModelMaker.cc
  ModelRightMenu.cc
  MouseEventHandler.cc
  SphereMaker.cc
  TimePanel.cc
  ToolsWidget.cc
  TopicSelector.cc
)

set (qt_headers
  Actions.hh
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
  TimePanel.hh
  ToolsWidget.hh
  TopicSelector.hh
)

set (headers
  qt.h
  qt_test.h
  BoxMaker.hh
  CylinderMaker.hh
  EntityMaker.hh
  GuiIface.hh
  GuiEvents.hh
  KeyEventHandler.hh
  LightMaker.hh
  ModelManipulator.hh
  MeshMaker.hh
  MouseEventHandler.hh
  ModelMaker.hh
  SphereMaker.hh
)

#if (HAVE_QWT)
#  set (sources ${sources}
#    Diagnostics.cc
#    IncrementalPlot.cc
#  )
#  set (qt_headers ${qt_headers}
#    Diagnostics.hh
#    IncrementalPlot.hh
#  )
#endif()

set (qt_tests
  DataLogger_TEST.cc
  TimePanel_TEST.cc
)

# MainWindow is using screen rendering
if (ENABLE_SCREEN_TESTS)
  set (qt_tests   ${qt_tests} MainWindow_TEST.cc)
endif()

# Generate executables for each of the QT unit tests
gz_build_qt_tests(${qt_tests})

set (resources resources.qrc)

QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

gz_add_executable(gzclient ${sources} main.cc ${headers_MOC} ${headers} ${resources_RCC})
gz_add_library(gazebo_gui ${sources} ${headers_MOC} ${headers} ${resources_RCC})

add_dependencies(gazebo_gui gazebo_msgs)
add_dependencies(gzclient gazebo_msgs)

target_link_libraries(gazebo_gui
  gazebo_common
  gazebo_transport
  gazebo_rendering
  gazebo_sensors
  gazebo_msgs
  gazebo_gui_building
  gazebo_gui_terrain
  gazebo_gui_viewers
  gzqtpropertybrowser
  libgazebo
  ${QT_LIBRARIES}
  ${ogre_libraries}
  ${Boost_LIBRARIES}
  pthread
)

target_link_libraries(gzclient
  gazebo_common
  gazebo_transport
  gazebo_rendering
  gazebo_sensors
  gazebo_msgs
  gazebo_gui_building
  gazebo_gui_terrain
  gazebo_gui_viewers
  gzqtpropertybrowser
  libgazebo
  ${QT_LIBRARIES}
  ${ogre_libraries}
  ${Boost_LIBRARIES}
  pthread
)

add_definitions(${QT_DEFINITIONS})

set (gui_headers "" CACHE INTERNAL "gui headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(gui_headers
    "GUI Headers" "#include \"gazebo/gui/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/gui.hh.in ${CMAKE_CURRENT_BINARY_DIR}/gui.hh )

gz_install_library(gazebo_gui)
gz_install_includes("gui" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/gui.hh)
gz_install_executable(gzclient)
