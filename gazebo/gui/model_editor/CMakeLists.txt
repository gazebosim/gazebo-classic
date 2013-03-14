include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

set (sources
  BuildingEditorPalette.cc
  BuildingEditorWidget.cc
  BuildingItem.cc
  BuildingMaker.cc
  BuildingModelManip.cc
  DoorItem.cc
  EditorEvents.cc
  EditorItem.cc
  EditorView.cc
  FinishBuildingDialog.cc
  FloorItem.cc
  GrabberHandle.cc
  GridLines.cc
  LevelInspectorDialog.cc
  LevelWidget.cc
  LineSegmentItem.cc
  PolylineItem.cc
  RectItem.cc
  RotateHandle.cc
  ScaleWidget.cc
  StairsInspectorDialog.cc
  StairsItem.cc
  WallInspectorDialog.cc
  WallItem.cc
  WindowDoorInspectorDialog.cc
  WindowItem.cc
)

set (qt_headers
  BuildingEditorPalette.hh
  BuildingModelManip.hh
  DoorItem.hh
  EditorItem.hh
  EditorView.hh
  FinishBuildingDialog.hh
  FloorItem.hh
  LevelInspectorDialog.hh
  LevelWidget.hh
  RectItem.hh
  ScaleWidget.hh
  StairsInspectorDialog.hh
  StairsItem.hh
  WallInspectorDialog.hh
  WallItem.hh
  WindowDoorInspectorDialog.hh
  WindowItem.hh
)

set (headers
  BuildingEditorWidget.hh
  BuildingItem.hh
  BuildingMaker.hh
  EditorEvents.hh
  GrabberHandle.hh
  GridLines.hh
  LineSegmentItem.hh
  PolylineItem.hh
  RotateHandle.hh
)

set (resources ../resources.qrc)
# Macro BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION defined to workaround
# QTBUG-22829 issue with qt < qt5 and boost 1.53
# https://bugreports.qt-project.org/browse/QTBUG-22829
QT4_WRAP_CPP(headers_MOC ${qt_headers}
             OPTIONS -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
QT4_ADD_RESOURCES(resources_RCC ${resources})

add_library(gazebo_model_editor STATIC ${sources} ${headers_MOC} ${headers} ${resources_RCC})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
