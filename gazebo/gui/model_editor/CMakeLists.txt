include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

set (sources
  EditorWidget.cc
  BuildingEditorWidget.cc
  BuildingEditorPalette.cc
  FinishModelDialog.cc
  WindowDoorInspectorDialog.cc
  WallInspectorDialog.cc
  GridLines.cc
  CornerGrabber.cc
  EditorView.cc
  RectItem.cc
  WindowItem.cc
  DoorItem.cc
  PolylineItem.cc
  WallItem.cc
)

set (qt_headers
  EditorWidget.hh
  BuildingEditorWidget.hh
  BuildingEditorPalette.hh
  FinishModelDialog.hh
  WindowDoorInspectorDialog.hh
  WallInspectorDialog.hh
  GridLines.hh
  CornerGrabber.hh
  EditorView.hh
  RectItem.hh
  WindowItem.hh
  DoorItem.hh
  PolylineItem.hh
  WallItem.hh
)

set (resources ../resources.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

add_library(gazebo_model_editor STATIC ${sources} ${headers_MOC} ${resources_RCC})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
