include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (${QT_USE_FILE})


set (sources
  JointInspector.cc
  JointMaker.cc
  ModelEditor.cc
  ModelCreator.cc
  ModelEditorPalette.cc
)

set (qt_headers
  JointInspector.hh
  ModelEditor.hh
  ModelEditorPalette.hh
)

set (headers
  JointMaker.hh
  ModelCreator.hh
)

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
