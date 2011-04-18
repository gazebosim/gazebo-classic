include (${gazebo_cmake_dir}/GazeboUtils.cmake)
include ( ${wxWidgets_USE_FILE} )

include_directories( 
  ${XML_INCLUDE_DIRS} 
  ${PROJECT_BINARY_DIR}/src/msgs
  ${PROJECT_SOURCE_DIR}/src 
  ${OGRE_INCLUDE_DIRS}
  ${wxWidgets_INCLUDE_DIRS}
  ${GTK2_INCLUDE_DIRS}
  propgrid 
  ${BOOST_INCLUDE_DIRS}
)

link_directories(  
  ${BOOST_LIBRARY_DIRS} 
)

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

set_source_files_properties(${sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}} ${wxWidgets_CXX_FLAGS}")

add_library(gazebo_gui SHARED ${sources})

target_link_libraries(gazebo_gui Xpm  ${wxWidgets_LIBRARIES} ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}} ${boost_libraries})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_gui PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

set_target_properties(gazebo_gui PROPERTIES OUTPUT_NAME "gazebo_gui" VERSION ${GAZEBO_VERSION})

install (TARGETS gazebo_gui DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo/gui COMPONENT headers)
