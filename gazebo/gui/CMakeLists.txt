include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include ( ${QT_USE_FILE} )

link_directories(
  ${Boost_LIBRARY_DIRS}
  ${tinyxml_LIBRARY_DIRS}
)

include_directories(
  ${CMAKE_SOURCE_DIR}/deps
  ${QWT_INCLUDE_DIR}
)

include_directories(${tinyxml_INCLUDE_DIRS})

add_definitions(${QT_DEFINITIONS})

add_subdirectory(qtpropertybrowser)

set (internal_qt_headers
  VideoRecorder.hh)

set (sources
  Actions.cc
  AlignWidget.cc
  ApplyWrenchDialog.cc
  CloneWindow.cc
  ConfigWidget.cc
  Conversions.cc
  DataLogger.cc
  Editor.cc
  EntityMaker.cc
  GLWidget.cc
  GuiEvents.cc
  GuiIface.cc
  InsertModelWidget.cc
  JointControlWidget.cc
  KeyEventHandler.cc
  LayersWidget.cc
  LightMaker.cc
  LogPlayWidget.cc
  MainWindow.cc
  ModelAlign.cc
  ModelListWidget.cc
  ModelMaker.cc
  ModelManipulator.cc
  ModelRightMenu.cc
  ModelSnap.cc
  MouseEventHandler.cc
  RenderWidget.cc
  SaveEntityDialog.cc
  SpaceNav.cc
  SplashScreen.cc
  TimePanel.cc
  TimeWidget.cc
  ToolsWidget.cc
  TopicSelector.cc
  TopToolbar.cc
  UserCmdHistory.cc
  VideoRecorder.cc
  ViewAngleWidget.cc
)

if (NOT USE_EXTERNAL_TINYXML)
  include_directories (${CMAKE_SOURCE_DIR}/deps/win/tinyxml)
  set (sources ${sources}
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinystr.cpp
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinyxml.cpp
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinyxmlerror.cpp
       ${CMAKE_SOURCE_DIR}/deps/win/tinyxml/tinyxmlparser.cpp)
endif()

set (qt_headers
  Actions.hh
  AlignWidget.hh
  ApplyWrenchDialog.hh
  CloneWindow.hh
  ConfigWidget.hh
  DataLogger.hh
  Editor.hh
  GLWidget.hh
  InsertModelWidget.hh
  JointControlWidget.hh
  LayersWidget.hh
  LogPlayWidget.hh
  MainWindow.hh
  ModelListWidget.hh
  ModelRightMenu.hh
  RenderWidget.hh
  SaveEntityDialog.hh
  SplashScreen.hh
  TimePanel.hh
  TimeWidget.hh
  ToolsWidget.hh
  TopicSelector.hh
  TopToolbar.hh
  UserCmdHistory.hh
  ViewAngleWidget.hh
)

set (headers
  qt.h
  qt_test.h
  Conversions.hh
  EntityMaker.hh
  GuiIface.hh
  GuiEvents.hh
  GuiPlugin.hh
  GuiTypes.hh
  KeyEventHandler.hh
  LightMaker.hh
  ModelAlign.hh
  ModelManipulator.hh
  ModelSnap.hh
  MouseEventHandler.hh
  ModelMaker.hh
  SpaceNav.hh
)

if (HAVE_OCULUS)
  set (sources ${sources}
    OculusWindow.cc
  )
  set (qt_headers ${qt_headers}
    OculusWindow.hh
  )
endif ()

set (qt_tests
  BuildingEditor_TEST.cc
  ConfigWidget_TEST.cc
  DataLogger_TEST.cc
  GLWidget_TEST.cc
  GLWidget_TEST2.cc
  GuiIface_TEST.cc
  LightMaker_TEST.cc
  ModelAlign_TEST.cc
  ModelListWidget_TEST.cc
  ModelMaker_TEST.cc
  ModelManipulator_TEST.cc
  RenderWidget_TEST.cc
  SaveEntityDialog_TEST.cc
  SplashScreen_TEST.cc
  TimePanel_TEST.cc
  TimeWidget_TEST.cc
  TopToolbar_TEST.cc
  ViewAngleWidget_TEST.cc
)

add_subdirectory(viewers)
add_subdirectory(terrain)
add_subdirectory(building)
add_subdirectory(model)
add_subdirectory(plot)

if (HAVE_GRAPHVIZ)
  add_definitions(-DWITH_CGRAPH)
  add_definitions(-DHAVE_STRING_H)
  # Activate Export macro for building the library itself
  # if QGVCORE_LIB is not set or equal 0 (null), import macro is used
  add_definitions(-DQGVCORE_LIB)
  include_directories(${GRAPHVIZ_INCLUDE_DIR})
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/qgv)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/qgv/private)
  add_subdirectory(qgv)
endif()

# MainWindow is using screen rendering
if (ENABLE_SCREEN_TESTS)
  set (qt_tests
      ${qt_tests}
      ApplyWrenchDialog_TEST.cc
      LaserVisualization_TEST.cc
      LayersWidget_TEST.cc
      MainWindow_TEST.cc
      ModelSnap_TEST.cc
      Projection_TEST.cc)
endif()

# Generate executables for each of the QT unit tests
gz_build_qt_tests(${qt_tests})

set (resources resources.qrc)

QT4_WRAP_CPP(headers_MOC ${qt_headers} ${internal_qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

gz_add_library(gazebo_gui ${sources} ${headers_MOC} ${resources_RCC})
set_target_properties(gazebo_gui PROPERTIES COMPILE_DEFINITIONS "QWT_DLL")
gz_add_executable(gzclient main.cc ${resources_RCC})

add_dependencies(gazebo_gui gazebo_msgs)
add_dependencies(gzclient gazebo_gui)

add_definitions(-DBUILDING_DLL_GZ_GUI)

target_link_libraries(gazebo_gui
  libgazebo_client
  gazebo_common
  gazebo_transport
  gazebo_rendering
  gazebo_msgs
  gzqtpropertybrowser
  ${QT_LIBRARIES}
  ${ogre_libraries}
  ${Boost_LIBRARIES}
  ${tinyxml_LIBRARIES}
  ${QWT_LIBRARY}
)

if (UNIX)
  target_link_libraries(gazebo_gui pthread)
endif()

if (HAVE_GRAPHVIZ)
  target_link_libraries(gazebo_gui ${GRAPHVIZ_LIBRARIES})
endif()

target_link_libraries(gzclient gazebo_gui)

if (UNIX)
  target_link_libraries(gzclient pthread)
endif()

if (SPNAV_LIBRARY)
  target_link_libraries(gazebo_gui ${SPNAV_LIBRARY})
  target_link_libraries(gzclient ${SPNAV_LIBRARY})
endif()

if (USE_PCH)
  add_pch(gazebo_gui gui_pch.hh)
endif()

set (gtest_sources
  Conversions_TEST.cc
)

gz_build_tests(${gtest_sources} EXTRA_LIBS gazebo_gui)

add_definitions(${QT_DEFINITIONS})

set (gui_headers "" CACHE INTERNAL "gui headers" FORCE)
foreach (hdr ${headers} ${qt_headers})
  APPEND_TO_CACHED_STRING(gui_headers
    "GUI Headers" "#include \"gazebo/gui/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/gui.hh.in ${CMAKE_CURRENT_BINARY_DIR}/gui.hh )

gz_install_library(gazebo_gui)
gz_install_includes("gui" ${headers} ${qt_headers} ${CMAKE_CURRENT_BINARY_DIR}/gui.hh)
gz_install_executable(gzclient)

if (UNIX)
  manpage(gzclient 1)
endif()
