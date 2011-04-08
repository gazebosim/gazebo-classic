include (${gazebo_cmake_dir}/GazeboUtils.cmake)

########################################
# Include all the search paths for headers
string (REPLACE " " ";" boost_include_dirs_split "${boost_include_dirs}")
string (REPLACE " " ";" assimp_include_dirs_split "${assimp_include_dirs}")
string (REPLACE " " ";" bullet_include_dirs_split "${bullet_include_dirs}")
string (REPLACE " " ";" threadpool_include_dirs_split "${threadpool_include_dirs}")
string (REPLACE " " ";" boost_library_dirs_split "${boost_library_dirs}")
string (REPLACE " " ";" assimp_library_dirs_split "${assimp_library_dirs}")
string (REPLACE " " ";" bullet_library_dirs_split "${bullet_library_dirs}")

include_directories( ${wxWidgets_INCLUDE_DIRS} )
link_directories( ${wxWidgets_LIBRARY_DIRS} )
include (${wxWidgets_USE_FILE})

include_directories(
  .
  ${CMAKE_SOURCE_DIR}/libgazebo 
  wx 
  physics 
  physics/ode
  physics/bullet
  #audio_video
  controllers 
  rendering 
  sensors 
  sensors/camera
  sensors/contact
  sensors/imu
  sensors/ir
  sensors/ray
  ${LIBAVCODEC_PATH}
  ${LIBAVFORMAT_PATH}
  ${libtool_include_path}
  ${gazeboserver_include_dirs}
  ${boost_include_dirs_split}
  ${freeimage_include_dir}
  ${bullet_include_dirs_split}
  ${threadpool_include_dirs_split}
  ${GTK2_INCLUDE_DIRS}
  ${assimp_include_dirs_split}
)

link_directories(  
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${CMAKE_BINARY_DIR}/server/wx 
 ${CMAKE_BINARY_DIR}/server/physics/bullet 
 ${boost_library_dirs_split} 
 ${assimp_library_dirs_split} 
 ${bullet_library_dirs_split} 
 ${freeimage_library_dir}
 ${gazeboserver_link_dirs} 
)

if (use_internal_assimp)
  include_directories( ${CMAKE_SOURCE_DIR}/3rd_party/assimp/include/assimp)
  link_directories( ${CMAKE_BINARY_DIR}/3rd_party/assimp )
endif (use_internal_assimp)


########################################
# All the library search paths
add_subdirectory(rendering)
add_subdirectory(physics)
add_subdirectory(sensors)
add_subdirectory(wx)
add_subdirectory(controllers)
add_subdirectory(transport)
#add_subdirectory(audio_video)

########################################
# Process all the subdirectories
SET (sources Common.cc
             Vector3.cc 
             Vector4.cc 
             Quatern.cc
             Pose3d.cc
             World.cc
             XMLConfig.cc
             GazeboConfig.cc
             Time.cc
             Entity.cc
             Exception.cc
             GazeboMessage.cc
             Model.cc
             Angle.cc
             Param.cc
             GraphicsIfaceHandler.cc
             SimulationIfaceHandler.cc
             Rand.cc
             FactoryIfaceHandler.cc
             Mesh.cc
             MeshManager.cc
             MeshLoader.cc
             OgreLoader.cc
             AssimpLoader.cc
             STLLoader.cc
             Plugin.cc
             Events.cc
             Event.cc
             Box.cc
             Plane.cc
             Messages.cc
             Matrix3.cc
             Matrix4.cc
             Timer.cc
             Diagnostics.cc
             Color.cc
)

SET (headers Common.hh
             Entity.hh
             Exception.hh
             Console.hh
             Global.hh
             Model.hh
             Pose3d.hh
             Quatern.hh
             SingletonT.hh
             StaticPluginRegister.hh
             Time.hh
             Vector2.hh
             Vector3.hh
             Vector4.hh
             World.hh
             XMLConfig.hh
             GazeboConfig.hh
             Angle.hh
             Param.hh
             GraphicsIfaceHandler.hh
             SimulationIfaceHandler.hh
             Simulator.hh
             Rand.hh
             FactoryIfaceHandler.hh
             Mesh.hh
             MeshManager.hh
             MeshLoader.hh
             OgreLoader.hh
             AssimpLoader.hh
             STLLoader.hh
             Events.hh
             Plugin.hh
             Messages.hh
             Box.hh
             Plane.hh
             Matrix3.hh
             Matrix4.hh
             Timer.hh
             Diagnostics.hh
             Color.hh
)

APPEND_TO_SERVER_HEADERS(${headers})
APPEND_TO_SERVER_SOURCES(${sources})

set (server_includes "" CACHE STRING "server includes auto generated" FORCE)
foreach (header ${gazeboserver_headers_nopath})
  set (server_includes "${server_includes} #include <gazebo/${header}>\n" CACHE STRING "server includes auto generated" FORCE)
endforeach (header ${gazeboserver_headers_nopath})

add_library(gazebo_server_library SHARED ${gazeboserver_sources} )
set_target_properties(gazebo_server_library PROPERTIES VERSION ${GAZEBO_VERSION})

add_executable(gazebo_server Simulator.cc server.cc)
set_target_properties(gazebo_server PROPERTIES SKIP_BUILD_RPATH TRUE)
add_executable(gazebo_client client.cc)
set_target_properties(gazebo_client PROPERTIES SKIP_BUILD_RPATH TRUE)

# add_executable(gazebo-exec main.cc)
# set_target_properties(gazebo-exec PROPERTIES OUTPUT_NAME "gazebo")

# Removes colon separated rpath. This breaks Ubuntu-intrepid 
# (possibly gcc-4.3.2).
#set_target_properties(gazebo-exec PROPERTIES SKIP_BUILD_RPATH TRUE)

LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")

set_source_files_properties(${gazeboserver_sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${gazeboserver_sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  #set_target_properties( gazebo-exec PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags} ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")
  set_target_properties( gazebo_server PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags} ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")
  set_target_properties( gazebo_client PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags} ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}}")
else (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  #set_target_properties( gazebo-exec PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
    set_target_properties( gazebo_server PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
    set_target_properties( gazebo_client PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})


#target_link_libraries( gazebo-exec ${libtool_library}
#                                   ${gazeboserver_link_libs} 
#                                   ${boost_libraries} 
#                                   ${assimp_libraries} 
#                                   ${freeimage_library} 
#                                   gazebo_transport
#                                   gazebo_server_library
#                                   gazebo_rendering
#                                   #gazebo_av-shared
#                                   gazebo_wx
#                                   gazebo_physics
#                                   gazebo_msgs
#                                   gazeboshm 
#                                   assimp
#                                   pthread
#                                   ${PROTOBUF_LIBRARY}
#                                   ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}}
#)

target_link_libraries( gazebo_server ${libtool_library}
                                   ${gazeboserver_link_libs} 
                                   ${boost_libraries} 
                                   ${assimp_libraries} 
                                   gazebo_transport
                                   gazebo_server_library
                                   gazebo_physics
                                   gazebo_msgs
                                   gazeboshm 
                                   assimp
                                   pthread
                                   ${PROTOBUF_LIBRARY}
                                   ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}}
)

target_link_libraries( gazebo_client ${libtool_library}
                                     ${gazeboserver_link_libs} 
                                     ${boost_libraries} 
                                     ${assimp_libraries} 
                                     ${freeimage_library} 
                                     gazebo_transport
                                     gazebo_server_library
                                     gazebo_rendering
                                     gazebo_wx
                                     gazebo_msgs
                                     gazeboshm 
                                     assimp
                                     pthread
                                     ${PROTOBUF_LIBRARY}
                                     ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}}
)

APPEND_TO_CACHED_LIST(gazeboserver_ldflags 
                      ${gazeboserver_ldflags_desc} 
                      -lassimp -lgazebo_physics -lgazebo_av 
                      -lgazebo_wx -lgazebo_rendering)


target_link_libraries( gazebo_server_library ${libtool_library} 
                                     ${freeimage_library} 
                                     ${PROTOBUF_LIBRARY}
                                     gazeboshm 
                                     gazebo_rendering 
                                     gazebo_physics 
                                     ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}})

if (INCLUDE_BULLET)
  target_link_libraries(gazebo_server_library ${bullet_lflags} gazebo_physics_bullet )
  #target_link_libraries(gazebo-exec ${bullet_lflags} gazebo_physics_bullet )
  target_link_libraries(gazebo_server ${bullet_lflags} gazebo_physics_bullet )

  APPEND_TO_CACHED_LIST(gazeboserver_ldflags 
                      ${gazeboserver_ldflags_desc} 
                      -lgazebo_physics_bullet)
endif (INCLUDE_BULLET)

if (INCLUDE_ODE)
  target_link_libraries(gazebo_server_library gazebo_physics_ode ${ODE_LIBRARIES})
  #target_link_libraries(gazebo-exec gazebo_physics_ode ${ODE_LIBRARIES})
  target_link_libraries(gazebo_server gazebo_physics_ode ${ODE_LIBRARIES})
  APPEND_TO_CACHED_LIST(gazeboserver_ldflags 
                      ${gazeboserver_ldflags_desc} 
                      -lgazebo_physics_ode)
endif (INCLUDE_ODE)

STRING (REPLACE ";" " " gazeboserver_cflags "${gazeboserver_cflags}")
STRING (REPLACE ";" " " gazeboserver_ldflags "${gazeboserver_ldflags}")

configure_file(${PROJECT_SOURCE_DIR}/server/gazeboserver.hh.in ${PROJECT_BINARY_DIR}/gazeboserver.hh)
install(FILES ${PROJECT_BINARY_DIR}/gazeboserver.hh DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo COMPONENT headers)

configure_file(${CMAKE_SOURCE_DIR}/cmake/gazeboserver_pkgconfig.cmake ${CMAKE_CURRENT_BINARY_DIR}/gazeboserver.pc @ONLY)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/gazeboserver.pc DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig COMPONENT pkgconfig)


install (TARGETS gazebo_server_library DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
#install (TARGETS gazebo-exec DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (TARGETS gazebo_server DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (TARGETS gazebo_client DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (FILES ${gazeboserver_headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo COMPONENT headers)


# INCLUDE DIRS
#${TBB_INCLUDE_DIRS}
#${ODE_INCLUDE_DIRS}
#${ogre_include_dirs}
#${XML_INCLUDE_DIRS}
#${XPM_INCLUDE_DIRS}
#${OAL_INCLUDE_DIRS}
#${AVF_INCLUDE_DIRS}
#${AVC_INCLUDE_DIRS})

# LINK DIRS
#${TBB_LIBRARY_DIRS}
#${ODE_LIBRARY_DIRS}
#${ogre_library_dirs}
#${GTK2_LIBRARY_DIRS} 
#${XML_LIBRARY_DIRS}
#${XPM_LIBRARY_DIRS}
#${OAL_LIBRARY_DIRS}
#${AVF_LIBRARY_DIRS}
#${AVC_LIBRARY_DIRS})

# LINK LIBS
#${TBB_LINK_LIBS} ${TBB_LIBRARIES}
#${ODE_LINK_LIBS} ${ODE_LIBRARIES}
#${ogre_link_libs} ${ogre_libraries}
#${GTK2_LINK_LIBS} ${GTK2_LIBRARIES}
#${XML_LINK_LIBS} ${XML_LIBRARIES}
#${XPM_LINK_LIBS} ${XPM_LIBRARIES}
#${OAL_LINK_LIBS} ${OAL_LIBRARIES}
#${AVF_LINK_LIBS} ${AVF_LIBRARIES}
#${AVC_LINK_LIBS} ${AVC_LIBRARIES}

# LD FLAGS
#${TBB_LDFLAGS}
#${ODE_LDFLAGS}
#${ogre_ldflags}
#${GTK2_LDFLAGS}
#${XML_LDFLAGS}

# C Flags
#${ogre_cflags})
#${GTK2_CFLAGS})
#${XML_CFLAGS})


# REMOVED FROM SEARCHFORSTUFF
  #APPEND_TO_CACHED_LIST(gazeboserver_ldflags
  #                      ${gazeboserver_ldflags_desc} 
  #                      ${bullet_lflags})


