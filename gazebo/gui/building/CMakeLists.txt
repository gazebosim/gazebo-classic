include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
  building/BaseInspectorDialog.cc
  building/BuildingEditor.cc
  building/BuildingEditorEvents.cc
  building/BuildingEditorPalette.cc
  building/BuildingEditorWidget.cc
  building/BuildingMaker.cc
  building/BuildingModelManip.cc
  building/DoorItem.cc
  building/EditorItem.cc
  building/EditorView.cc
  building/FloorItem.cc
  building/GrabberHandle.cc
  building/GridLines.cc
  building/ImportImageDialog.cc
  building/ImportImageView.cc
  building/LevelInspectorDialog.cc
  building/LevelWidget.cc
  building/MeasureItem.cc
  building/RectItem.cc
  building/RotateHandle.cc
  building/ScaleWidget.cc
  building/SegmentItem.cc
  building/StairsInspectorDialog.cc
  building/StairsItem.cc
  building/WallInspectorDialog.cc
  building/WallSegmentItem.cc
  building/WindowDoorInspectorDialog.cc
  building/WindowItem.cc
  PARENT_SCOPE
)

set (internal_qt_headers ${internal_qt_headers}
  building/BaseInspectorDialog.hh
  building/BuildingEditor.hh
  building/BuildingEditorPalette.hh
  building/BuildingModelManip.hh
  building/DoorItem.hh
  building/EditorItem.hh
  building/EditorView.hh
  building/FloorItem.hh
  building/ImportImageDialog.hh
  building/ImportImageView.hh
  building/LevelInspectorDialog.hh
  building/LevelWidget.hh
  building/MeasureItem.hh
  building/RectItem.hh
  building/ScaleWidget.hh
  building/StairsInspectorDialog.hh
  building/StairsItem.hh
  building/WallInspectorDialog.hh
  building/WallSegmentItem.hh
  building/WindowDoorInspectorDialog.hh
  building/WindowItem.hh
  PARENT_SCOPE
)

set (qt_tests_local
  BuildingMaker_TEST.cc
)

gz_build_qt_tests(${qt_tests_local})
