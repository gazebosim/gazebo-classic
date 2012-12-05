include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${QT_USE_FILE} )

set (sources
  EditorWidget.cc
  BuildingCreatorWidget.cc
  BuildingCreatorPalette.cc
  FinishModelDialog.cc
  WindowDoorInspectorDialog.cc
  WallInspectorDialog.cc
  SelectableLineSegment.cc
  GridLines.cc
  CornerGrabber.cc
  CreatorView.cc
  RectItem.cc
  WindowItem.cc
  DoorItem.cc
)

set (qt_headers
  EditorWidget.hh
  BuildingCreatorWidget.hh
  BuildingCreatorPalette.hh
  FinishModelDialog.hh
  WindowDoorInspectorDialog.hh
  WallInspectorDialog.hh
  SelectableLineSegment.hh
  GridLines.hh
  CornerGrabber.hh
  CreatorView.hh
  RectItem.hh
  WindowItem.hh
  DoorItem.hh
)

#set (resources qtpropertybrowser.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
#QT4_ADD_RESOURCES(resources_RCC ${resources})

add_library(gazebo_modelcreator STATIC ${sources} ${headers_MOC} )

include_directories(${CMAKE_CURRENT_BINARY_DIR})
