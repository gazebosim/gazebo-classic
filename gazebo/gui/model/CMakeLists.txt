include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (${QT_USE_FILE})


set (sources
  ImportDialog.cc
  JointInspector.cc
  JointMaker.cc
  ModelCreator.cc
  ModelData.cc
  ModelEditor.cc
  ModelEditorEvents.cc
  ModelEditorPalette.cc
  PartCollisionConfig.cc
  PartGeneralConfig.cc
  PartInspector.cc
  PartVisualConfig.cc
)

set (qt_headers
  ImportDialog.hh
  JointInspector.hh
  JointMaker.hh
  ModelCreator.hh
  ModelData.hh
  ModelEditor.hh
  ModelEditorPalette.hh
  PartVisualConfig.hh
  PartCollisionConfig.hh
  PartGeneralConfig.hh
  PartInspector.hh
  PartVisualConfig.hh
)

set (headers
  ModelEditorEvents.hh
)

# MainWindow is using screen rendering
if (ENABLE_SCREEN_TESTS)
  set (qt_tests ${qt_tests} JointMaker_TEST.cc)
endif()

# Generate executables for each of the QT unit tests
gz_build_qt_tests(${qt_tests})

set (resources ../resources.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

gz_add_library(gazebo_gui_model ${sources} ${headers_MOC} ${headers} ${resources_RCC})

target_link_libraries(gazebo_gui_model
  gazebo_common
#  gazebo_transport
#  gazebo_sdf_interface
  gazebo_rendering
  gazebo_msgs)

gz_install_library(gazebo_gui_model)
