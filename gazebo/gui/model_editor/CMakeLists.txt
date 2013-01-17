include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

set (sources
  BuildingEditorPalette.cc
  BuildingEditorWidget.cc
  BuildingItem.cc
  BuildingMaker.cc
  DoorItem.cc
  EditorEvents.cc
  EditorItem.cc
  EditorView.cc
  EditorWidget.cc
  FinishModelDialog.cc
  FloorItem.cc
  GrabberHandle.cc
  GridLines.cc
  LevelInspectorDialog.cc
  LevelWidget.cc
  LineSegmentItem.cc
  ModelManip.cc
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
  DoorItem.hh
  EditorItem.hh
  EditorView.hh
  FinishModelDialog.hh
  FloorItem.hh
  LevelInspectorDialog.hh
  LevelWidget.hh
  ModelManip.hh
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
  EditorWidget.hh
  GrabberHandle.hh
  GridLines.hh
  LineSegmentItem.hh
  PolylineItem.hh
  RotateHandle.hh
)

set (resources ../resources.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

add_library(gazebo_model_editor STATIC ${sources} ${headers_MOC} ${headers} ${resources_RCC})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
