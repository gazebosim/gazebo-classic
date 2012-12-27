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
  EditorView.cc
  RectItem.cc
  BuildingItem.cc
  WindowItem.cc
  DoorItem.cc
  PolylineItem.cc
  WallItem.cc
  LineSegmentItem.cc
  BuildingMaker.cc
  EditorEvents.cc
  EditorItem.cc
  StairsItem.cc
  LevelWidget.cc
)

set (qt_headers
  BuildingEditorPalette.hh
  FinishModelDialog.hh
  WindowDoorInspectorDialog.hh
  WallInspectorDialog.hh
  StairsInspectorDialog.hh
  LevelInspectorDialog.hh
  BuildingMaker.hh
  EditorItem.hh
  LevelWidget.hh
  EditorView.hh
)

set (headers
  BuildingEditorWidget.hh
  EditorWidget.hh
  CornerGrabber.hh
  RotateHandle.hh
  BuildingItem.hh
  WindowItem.hh
  DoorItem.hh
  RectItem.hh
  LineSegmentItem.hh
  GridLines.hh
  PolylineItem.hh
  WallItem.hh
  StairsItem.hh
  EditorEvents.hh
)

set (resources ../resources.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

add_library(gazebo_model_editor STATIC ${sources} ${headers_MOC} ${headers} ${resources_RCC})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
