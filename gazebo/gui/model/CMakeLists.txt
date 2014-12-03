include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (${QT_USE_FILE})


set (sources
  ImportDialog.cc
  JointInspector.cc
  JointMaker.cc
  ModelCreator.cc
  ModelEditor.cc
  ModelEditorEvents.cc
  ModelEditorPalette.cc
)

set (qt_headers
  ImportDialog.hh
  JointInspector.hh
  JointMaker.hh
  ModelCreator.hh
  ModelEditor.hh
  ModelEditorPalette.hh
)

set (headers
  ModelEditorEvents.hh
)

set (resources ../resources.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_GUI_MODEL")

if (WIN32)
  add_library(gazebo_gui_model STATIC ${sources} ${headers_MOC} ${headers} ${resources_RCC})
  target_link_libraries(gazebo_gui_model ${general_libraries})
else()
  gz_add_library(gazebo_gui_model ${sources} ${headers_MOC} ${headers} ${resources_RCC})
endif()

target_link_libraries(gazebo_gui_model
  gazebo_common
#  gazebo_transport
#  gazebo_sdf_interface
  gazebo_rendering
  gazebo_msgs
  ${QT_LIBRARIES})

gz_install_library(gazebo_gui_model)
