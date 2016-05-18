include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
  terrain/TerrainEditor.cc
  terrain/TerrainEditorPalette.cc
  PARENT_SCOPE
)

set (internal_qt_headers ${internal_qt_headers}
  terrain/TerrainEditor.hh
  terrain/TerrainEditorPalette.hh
  PARENT_SCOPE
)
