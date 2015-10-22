include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources_local
  model/CollisionConfig.cc
  model/ExtrudeDialog.cc
  model/ImportDialog.cc
  model/JointCreationDialog.cc
  model/JointInspector.cc
  model/JointMaker.cc
  model/LinkConfig.cc
  model/LinkInspector.cc
  model/ModelCreator.cc
  model/ModelData.cc
  model/ModelEditor.cc
  model/ModelEditorEvents.cc
  model/ModelEditorPalette.cc
  model/ModelPluginInspector.cc
  model/VisualConfig.cc
)

set (qt_headers_local
  model/CollisionConfig.hh
  model/ExtrudeDialog.hh
  model/ImportDialog.hh
  model/JointCreationDialog.hh
  model/JointInspector.hh
  model/JointMaker.hh
  model/LinkConfig.hh
  model/LinkInspector.hh
  model/ModelCreator.hh
  model/ModelData.hh
  model/ModelEditor.hh
  model/ModelEditorPalette.hh
  model/ModelPluginInspector.hh
  model/VisualConfig.hh
)

set (headers_install
  ModelEditorEvents.hh
  ModelEditor.hh
)

set (qt_tests_local
  CollisionConfig_TEST.cc
  JointInspector_TEST.cc
  LinkInspector_TEST.cc
  ModelPluginInspector_TEST.cc
  VisualConfig_TEST.cc
)

if (ENABLE_SCREEN_TESTS)
  set (qt_tests_local
    ${qt_tests_local}
    ExtrudeDialog_TEST.cc
    JointMaker_TEST.cc
    ModelCreator_TEST.cc
    ModelData_TEST.cc
    ModelEditor_TEST.cc
    ModelEditorPalette_TEST.cc
  )
endif()

if (HAVE_GRAPHVIZ)
  set (qt_headers_local ${qt_headers_local}
     model/GraphScene.hh
     model/GraphView.hh
     model/SchematicViewWidget.hh
  )

  set (sources_local ${sources_local}
    model/GraphScene.cc
    model/GraphView.cc
    model/SchematicViewWidget.cc
  )

  if (ENABLE_SCREEN_TESTS)
    set (qt_tests_local ${qt_tests_local}
      SchematicViewWidget_TEST.cc
    )
  endif()

endif()

set (sources ${sources} ${sources_local} PARENT_SCOPE)
set (internal_qt_headers ${internal_qt_headers} ${qt_headers_local} PARENT_SCOPE)

gz_build_qt_tests(${qt_tests_local})
gz_install_includes("gui/model" ${headers_install})
