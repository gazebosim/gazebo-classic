include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
  terrain/TerrainEditor.cc
  terrain/TerrainEditorPalette.cc
  PARENT_SCOPE
)

set (qt_headers ${qt_headers}
  terrain/TerrainEditor.hh
  terrain/TerrainEditorPalette.hh
  PARENT_SCOPE
)
