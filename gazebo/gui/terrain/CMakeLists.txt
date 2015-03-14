include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources
  TerrainEditor.cc
  TerrainEditorPalette.cc
)

set (qt_headers
  TerrainEditor.hh
  TerrainEditorPalette.hh
)

set (headers
)

set (resources ../resources.qrc)
QT5_WRAP_CPP(headers_MOC ${qt_headers})
QT5_ADD_RESOURCES(resources_RCC ${resources})

add_library(gazebo_gui_terrain STATIC ${sources} ${headers_MOC}
  ${headers} ${resources_RCC})

set_target_properties(gazebo_gui_terrain PROPERTIES POSITION_INDEPENDENT_CODE ON)
add_dependencies(gazebo_gui_terrain gazebo_msgs)
include_directories(${CMAKE_CURRENT_BINARY_DIR})
