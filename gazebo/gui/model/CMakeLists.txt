if (HAVE_GRAPHVIZ)
  add_subdirectory(qgv)
endif()

include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include (${QT_USE_FILE})


set (sources
  CollisionConfig.cc
  ExtrudeDialog.cc
  ImportDialog.cc
  JointInspector.cc
  JointMaker.cc
  LinkConfig.cc
  LinkInspector.cc
  ModelCreator.cc
  ModelData.cc
  ModelEditor.cc
  ModelEditorEvents.cc
  ModelEditorPalette.cc
  VisualConfig.cc
)

set (qt_headers
  CollisionConfig.hh
  ExtrudeDialog.hh
  ImportDialog.hh
  JointInspector.hh
  JointMaker.hh
  LinkConfig.hh
  LinkInspector.hh
  ModelCreator.hh
  ModelData.hh
  ModelEditor.hh
  ModelEditorPalette.hh
  VisualConfig.hh
)

set (headers
  ModelEditorEvents.hh
)

set (install_headers
  ModelEditor.hh
)

if (ENABLE_SCREEN_TESTS)
  set (qt_tests ${qt_tests}
    JointMaker_TEST.cc
    ExtrudeDialog_TEST.cc
    ModelCreator_TEST.cc
    ModelEditor_TEST.cc
    ModelEditorPalette_TEST.cc
  )
endif()

set (libraries_to_link
  gazebo_rendering
  gazebo_msgs
  gazebo_common
)

if (HAVE_GRAPHVIZ)
  set (qt_headers
    ${qt_headers}
    GraphScene.hh
    GraphView.hh
    SchematicViewWidget.hh
  )

  set (sources
    ${sources}
    GraphScene.cc
    GraphView.cc
    SchematicViewWidget.cc
  )

  set (qt_tests
    ${qt_tests}
    SchematicViewWidget_TEST.cc
  )

  set (libraries_to_link
    ${libraries_to_link}
    gazebo_gui_model_qgv
  )
endif()

# Generate executables for each of the QT unit tests
gz_build_qt_tests(${qt_tests})

set (resources ../resources.qrc)
QT4_WRAP_CPP(headers_MOC ${qt_headers})
QT4_ADD_RESOURCES(resources_RCC ${resources})

include_directories(${CMAKE_CURRENT_BINARY_DIR})

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_GZ_GUI_MODEL")

if (WIN32)
  add_library(gazebo_gui_model STATIC ${sources} ${headers_MOC} ${headers} ${resources_RCC})
  target_link_libraries(gazebo_gui_model ${general_libraries})
else()
  gz_add_library(gazebo_gui_model ${sources} ${headers_MOC} ${headers} ${resources_RCC})
endif()

target_link_libraries(gazebo_gui_model
  ${libraries_to_link}
)

gz_install_library(gazebo_gui_model)
gz_install_includes("gui/model" ${install_headers})
