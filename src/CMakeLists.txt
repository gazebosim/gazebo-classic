add_definitions(${tinyxml_cflags})

include_directories(SYSTEM
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${tinyxml_include_dirs}
  ${PROTOBUF_INCLUDE_DIR}
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src 
  ${PROJECT_BINARY_DIR}/src 
)

link_directories(  
  ${OGRE_LIBRARY_PATH}
  ${BOOST_LIBRARY_DIRS} 
  ${tinyxml_library_dirs}
)

add_subdirectory(msgs)
add_subdirectory(math)
add_subdirectory(sdf)
add_subdirectory(transport)
add_subdirectory(common)
add_subdirectory(rendering)
add_subdirectory(gui)
add_subdirectory(physics)
add_subdirectory(sensors)

add_dependencies(gazebo_physics gazebo_msgs)
add_dependencies(gazebo_physics_ode gazebo_msgs)
add_dependencies(gazebo_gui gazebo_msgs)
add_dependencies(gazebo_rendering gazebo_msgs)
add_dependencies(gazebo_sensors gazebo_rendering)
if(HAVE_BULLET)
  add_dependencies(gazebo_physics_bullet gazebo_msgs)
endif()

gz_add_executable(gzmaster master_main.cc Master.cc gazebo.cc)
gz_add_executable(gzserver server_main.cc Server.cc Master.cc gazebo.cc)
gz_add_executable(gazebo gazebo_main.cc Server.cc Master.cc gazebo.cc)


target_link_libraries(gzmaster gazebo_common
                               gazebo_transport
                               gazebo_sdf_interface
                               gazebo_msgs
                               ${boost_libraries}
                               ${freeimage_library}
                               ${PROTOBUF_LIBRARY})

target_link_libraries(gzserver gazebo_common
                               gazebo_sdf_interface
                               gazebo_transport
                               gazebo_physics
                               gazebo_sensors
                               gazebo_rendering
                               gazebo_msgs
                               pthread
                               ${tinyxml_libraries}
                               boost_program_options)

target_link_libraries(gazebo gazebo_common
                             gazebo_sdf_interface
                             gazebo_transport
                             gazebo_physics
                             gazebo_sensors
                             gazebo_rendering
                             gazebo_msgs
                             gazebo_gui
                             pthread
                             ${tinyxml_libraries}
                             boost_program_options)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "CHECK")
  target_link_libraries(gzserver gcov)
endif()


INSTALL_EXECUTABLE(gzmaster)
INSTALL_EXECUTABLE(gzserver)
INSTALL_EXECUTABLE(gazebo)

gz_add_library(libgazebo Server.cc Master.cc gazebo.cc)
set_target_properties(libgazebo PROPERTIES OUTPUT_NAME "gazebo")

target_link_libraries(libgazebo gazebo_common
                                gazebo_transport
                                gazebo_physics
                                gazebo_sensors
                                gazebo_rendering
                                gazebo_sdf_interface
                                gazebo_msgs
                                gazebo_math
                                pthread)

add_dependencies(libgazebo gazebo_msgs)
INSTALL_LIBRARY(libgazebo)

set(headers Server.hh Master.hh gazebo.hh gazebo_core.hh)
INSTALL_INCLUDES("" ${headers})
