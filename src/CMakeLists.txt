include ( ${wxWidgets_USE_FILE} )

include_directories( 
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${wxWidgets_INCLUDE_DIRS}
  ${XML_INCLUDE_DIRS} 
  ${PROJECT_BINARY_DIR}/src/msgs
  ${PROJECT_SOURCE_DIR}/src 
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

add_dependencies( gazebo_gui gazebo_msgs )
add_dependencies( gazebo_rendering gazebo_msgs )

add_executable(gazebo_server server.cc PhysicsServer.cc)
add_executable(gazebo_client client.cc GuiClient.cc)

#set_source_files_properties(server.cc client.cc PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")

#if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
#  set_target_properties( gazebo_server PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags} ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")
#  set_target_properties( gazebo_client PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags} ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")
#else (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
#    set_target_properties( gazebo_server PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
#    set_target_properties( gazebo_client PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
#endif ()

set_target_properties( gazebo_client PROPERTIES LINK_FLAGS "${ogre_ldflags}")

target_link_libraries( gazebo_server gazebo_common
                                     gazebo_transport
                                     gazebo_physics
                                     gazebo_msgs
                                     assimp
                                     pthread
                                     ${XML_LIBRARIES}
                                     ${libtool_library}
                                     ${TBB_LIBRARIES}
                                     ${ODE_LIBRARIES}
                                     ${boost_libraries}
                                     ${freeimage_library}
                                     ${PROTOBUF_LIBRARY})

target_link_libraries( gazebo_client gazebo_common
                                     gazebo_transport
                                     gazebo_gui
                                     gazebo_rendering
                                     gazebo_msgs
                                     pthread
                                     ${assimp_libraries}
                                     ${OPENGL_LIBRARIES}
                                     ${XML_LIBRARIES}
                                     ${boost_libraries}
                                     ${ogre_libraries}
                                     ${ogre_link_libs}
                                     ${PROTOBUF_LIBRARY})

install (TARGETS gazebo_server DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (TARGETS gazebo_client DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
