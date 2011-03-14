include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories( propgrid )

set (sources SimulationApp.cc
             SimulationFrame.cc
             RenderPanel.cc
             RenderControl.cc
             TimePanel.cc
             EntityMaker.cc
             BoxMaker.cc
             SphereMaker.cc
             CylinderMaker.cc
             PointLightMaker.cc
             SpotLightMaker.cc
             DirectionalLightMaker.cc
             MeshBrowser.cc
             ModelBuilder.cc
             GridPage.cc
             ParamsNotebook.cc
             FogPage.cc
             PhysicsPage.cc
             RenderingPage.cc
             Properties.cc
             DiagnosticsDialog.cc
             PlotPanel.cc
             propgrid/advprops.cpp
             propgrid/editors.cpp
             propgrid/extras.cpp
             propgrid/manager.cpp
             propgrid/odcombo.cpp
             propgrid/propgrid.cpp
             propgrid/props.cpp
             propgrid/xh_propgrid.cpp
)

set (headers SimulationApp.hh
             SimulationFrame.hh
             RenderPanel.hh
             RenderControl.hh
             TimePanel.hh
             EntityMaker.hh
             BoxMaker.hh
             SphereMaker.hh
             CylinderMaker.hh
             PointLightMaker.hh
             SpotLightMaker.hh
             DirectionalLightMaker.hh
             MeshBrowser.hh
             ModelBuilder.hh
             GridPage.hh
             ParamsNotebook.hh
             FogPage.hh
             PhysicsPage.hh
             RenderingPage.hh
             Properties.hh
             DiagnosticsDialog.hh
             PlotPanel.hh
)

LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}} ${wxWidgets_CXX_FLAGS}")

add_library(gazebo_wx SHARED ${sources})

target_link_libraries(gazebo_wx ${wxWidgets_LIBRARIES} Xpm ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_wx PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

set_target_properties(gazebo_wx PROPERTIES OUTPUT_NAME "gazebo_wx" VERSION ${GAZEBO_VERSION})

install (TARGETS gazebo_wx DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
