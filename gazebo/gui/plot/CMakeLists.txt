include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include_directories(${QWT_WIN_INCLUDE_DIR})

set (sources_local
  plot/EditableLabel.cc
  plot/ExportDialog.cc
  plot/IncrementalPlot.cc
  plot/IntrospectionCurveHandler.cc
  plot/Palette.cc
  plot/PlotCanvas.cc
  plot/PlotCurve.cc
  plot/PlotManager.cc
  plot/PlotTracker.cc
  plot/PlotWindow.cc
  plot/TopicCurveHandler.cc
  plot/VariablePill.cc
  plot/VariablePillContainer.cc
)

set (qt_headers_local
  plot/EditableLabel.hh
  plot/ExportDialog.hh
  plot/IncrementalPlot.hh
  plot/PlotCanvas.hh
  plot/Palette.hh
  plot/PlotWindow.hh
  plot/VariablePill.hh
  plot/VariablePillContainer.hh
)

set (headers_install
)

set (qt_tests_local
  EditableLabel_TEST.cc
  ExportDialog_TEST.cc
  IntrospectionCurveHandler_TEST.cc
  IncrementalPlot_TEST.cc
  Palette_TEST.cc
  PlotCanvas_TEST.cc
  PlotCurve_TEST.cc
  PlotWindow_TEST.cc
  TopicCurveHandler_TEST.cc
  VariablePill_TEST.cc
  VariablePillContainer_TEST.cc
)

if (ENABLE_SCREEN_TESTS)
  set (qt_tests_local
    ${qt_tests_local}

  )
endif()

set (sources ${sources} ${sources_local} PARENT_SCOPE)
set (internal_qt_headers ${internal_qt_headers} ${qt_headers_local} PARENT_SCOPE)

gz_build_qt_tests(${qt_tests_local})
gz_install_includes("gui/plot" ${headers_install})
