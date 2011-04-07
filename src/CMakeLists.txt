include ( ${wxWidgets_USE_FILE} )

include_directories( 
  ${PROJECT_SOURCE_DIR}/src 
  ${PROJECT_BINARY_DIR}/src 
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${wxWidgets_INCLUDE_DIRS}
  ${XML_INCLUDE_DIRS} 
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
)

link_directories(  
  ${OGRE_LIBRARY_PATH}
  ${BOOST_LIBRARY_DIRS} 
  ${assimp_library_dirs} 
  ${ODE_LIBRARY_DIRS}
)

add_subdirectory(msgs)
add_subdirectory(transport)
add_subdirectory(common)
add_subdirectory(libgazebo)
add_subdirectory(rendering)
add_subdirectory(gui)
add_subdirectory(physics)
add_subdirectory(sensors)

add_dependencies( gazebo_common gazebo_msgs )
add_dependencies( gazebo_gui gazebo_msgs )
add_dependencies( gazebo_rendering gazebo_msgs )
add_dependencies( gazebo_sensors gazebo_rendering )

add_executable(gzserver physics_server.cc PhysicsServer.cc)
add_executable(gzclient gui_client.cc GuiClient.cc)
add_executable(gzmaster master.cc Master.cc)
add_executable(gzsensor sensor_server.cc SensorServer.cc)
add_executable(gzcombined combined.cc Combined.cc)

target_link_libraries( gzmaster gazebo_common
                                gazebo_transport
                                gazebo_msgs
                                ${XML_LIBRARIES}
                                ${boost_libraries}
                                ${freeimage_library}
                                ${assimp_libraries}
                                ${PROTOBUF_LIBRARY})

target_link_libraries( gzcombined gazebo_common
                                  gazebo_transport
                                  gazebo_physics
                                  gazebo_sensors
                                  gazebo_rendering
                                  gazebo_msgs
                                  assimp
                                  pthread)


target_link_libraries( gzserver gazebo_common
                                gazebo_transport
                                gazebo_physics
                                gazebo_msgs
                                assimp
                                pthread)

target_link_libraries( gzclient gazebo_common
                                gazebo_transport
                                gazebo_gui
                                gazebo_rendering
                                gazebo_msgs
                                pthread)

target_link_libraries( gzsensor gazebo_common
                                gazebo_transport
                                gazebo_sensors
                                gazebo_rendering
                                gazebo_msgs
                                pthread)
                                

install (TARGETS gzmaster DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (TARGETS gzserver DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (TARGETS gzclient DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (TARGETS gzsensor DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (TARGETS gzcombined DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
