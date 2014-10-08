include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (${QT_USE_FILE})

set (sources
  BuildingEditor.cc
  BuildingEditorEvents.cc
  BuildingEditorPalette.cc
  BuildingEditorWidget.cc
  BuildingItem.cc
  BuildingMaker.cc
  BuildingModelManip.cc
  DoorItem.cc
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
  BuildingEditor.hh
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
  BuildingEditorEvents.hh
  BuildingEditorWidget.hh
  BuildingItem.hh
  BuildingMaker.hh
  GrabberHandle.hh
  GridLines.hh
  LineSegmentItem.hh
  PolylineItem.hh
  RotateHandle.hh
)

set (resources ../resources.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

gz_add_library(gazebo_gui_building ${sources} ${headers_MOC} ${headers} ${resources_RCC})

target_link_libraries(gazebo_gui_building
  gazebo_common
  gazebo_transport
  gazebo_rendering
  gazebo_msgs
  ${QT_LIBRARIES})

gz_install_library(gazebo_gui_building)
