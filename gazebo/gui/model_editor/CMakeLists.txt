include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

set (sources
  EditorWidget.cc
  BuildingEditorWidget.cc
  BuildingEditorPalette.cc
  FinishModelDialog.cc
  WindowDoorInspectorDialog.cc
  WallInspectorDialog.cc
  StairsInspectorDialog.cc
  LevelInspectorDialog.cc
  GridLines.cc
  CornerGrabber.cc
  RotateHandle.cc
  EditorItem.cc
  BuildingItem.cc
  RectItem.cc
  FloorItem.cc
  WindowItem.cc
  DoorItem.cc
  LineSegmentItem.cc
  PolylineItem.cc
  WallItem.cc
  BuildingMaker.cc
  EditorEvents.cc
  StairsItem.cc
  LevelWidget.cc
  ModelManip.cc
  EditorView.cc
)

set (qt_headers
  BuildingEditorPalette.hh
  FinishModelDialog.hh
  WindowDoorInspectorDialog.hh
  WallInspectorDialog.hh
  StairsInspectorDialog.hh
  LevelInspectorDialog.hh
  ModelManip.hh
  EditorItem.hh
  LevelWidget.hh
  RectItem.hh
  StairsItem.hh
  WallItem.hh
  DoorItem.hh
  WindowItem.hh
  EditorView.hh
)

set (headers
  BuildingMaker.hh
  BuildingEditorWidget.hh
  EditorWidget.hh
  CornerGrabber.hh
  RotateHandle.hh
  BuildingItem.hh
  FloorItem.hh
  LineSegmentItem.hh
  GridLines.hh
  PolylineItem.hh
  EditorEvents.hh
)

set (resources ../resources.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

add_library(gazebo_model_editor STATIC ${sources} ${headers_MOC} ${headers} ${resources_RCC})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
