INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

########################################
# Include all the search paths for headers
STRING (REPLACE " " ";" boost_include_dirs_split "${boost_include_dirs}")
STRING (REPLACE " " ";" assimp_include_dirs_split "${assimp_include_dirs}")
STRING (REPLACE " " ";" bullet_include_dirs_split "${bullet_include_dirs}")
STRING (REPLACE " " ";" threadpool_include_dirs_split "${threadpool_include_dirs}")
STRING (REPLACE " " ";" boost_library_dirs_split "${boost_library_dirs}")
STRING (REPLACE " " ";" assimp_library_dirs_split "${assimp_library_dirs}")
STRING (REPLACE " " ";" bullet_library_dirs_split "${bullet_library_dirs}")



INCLUDE_DIRECTORIES(
  .
  ${CMAKE_SOURCE_DIR}/libgazebo 
  gui 
  physics 
  physics/ode
  physics/bullet
  audio_video
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
  ${assimp_include_dirs_split}
  ${freeimage_include_dir}
  ${bullet_include_dirs_split}
  ${threadpool_include_dirs_split}
  ${FLTK_INCLUDE_DIR}
)

LINK_DIRECTORIES(  
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${CMAKE_BINARY_DIR}/server/gui 
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
ADD_SUBDIRECTORY(rendering)
ADD_SUBDIRECTORY(physics)
ADD_SUBDIRECTORY(sensors)
ADD_SUBDIRECTORY(gui)
ADD_SUBDIRECTORY(controllers)
ADD_SUBDIRECTORY(audio_video)

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
             Timer.cc
             Entity.cc
             GazeboError.cc
             GazeboMessage.cc
             Model.cc
             Angle.cc
             Param.cc
             GraphicsIfaceHandler.cc
             Simulator.cc
             Rand.cc
             Factory.cc
             Mesh.cc
             MeshManager.cc
             MeshLoader.cc
             OgreLoader.cc
             AssimpLoader.cc
             STLLoader.cc
)

SET (headers Common.hh
             Entity.hh
             GazeboError.hh
             GazeboMessage.hh
             Global.hh
             Model.hh
             Pose3d.hh
             Quatern.hh
             SingletonT.hh
             StaticPluginRegister.hh
             Time.hh
             Timer.hh
             Vector2.hh
             Vector3.hh
             Vector4.hh
             World.hh
             XMLConfig.hh
             GazeboConfig.hh
             Angle.hh
             Param.hh
             GraphicsIfaceHandler.hh
             Simulator.hh
             Rand.hh
             Factory.hh
             Mesh.hh
             MeshManager.hh
             MeshLoader.hh
             OgreLoader.hh
             AssimpLoader.hh
             STLLoader.hh
)

APPEND_TO_SERVER_HEADERS(${headers})
APPEND_TO_SERVER_SOURCES(${sources})

ADD_LIBRARY(gazebo_server SHARED ${gazeboserver_sources})
SET_TARGET_PROPERTIES(gazebo_server PROPERTIES VERSION ${GAZEBO_VERSION})

ADD_EXECUTABLE(gazebo-exec main.cc)
SET_TARGET_PROPERTIES(gazebo-exec PROPERTIES OUTPUT_NAME "gazebo")

# Removes colon separated rpath. This breaks Ubuntu-intrepid 
# (possibly gcc-4.3.2).
SET_TARGET_PROPERTIES(gazebo-exec PROPERTIES SKIP_BUILD_RPATH TRUE)

LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")

SET_SOURCE_FILES_PROPERTIES(${gazeboserver_sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")

IF (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  SET_SOURCE_FILES_PROPERTIES(${gazeboserver_sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
ENDIF (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

SET_TARGET_PROPERTIES( gazebo-exec PROPERTIES LINK_FLAGS "${LINK_FLAGS} ${gazebo_lflags}")

target_link_libraries( gazebo-exec ${libtool_library}
                                   ${gazeboserver_link_libs} 
                                   ${boost_libraries} 
                                   ${assimp_libraries} 
                                   gazebo_server
                                   ${freeimage_library} 
                                   gazebo_rendering
                                   gazebo_av-shared
                                   gazebo_gui
                                   gazebo_physics
                                   gazebo 
                                   assimp
)

target_link_libraries( gazebo_server ${libtool_library} ${freeimage_library} gazebo gazebo_rendering gazebo_physics )

if (INCLUDE_BULLET)
  target_link_libraries(gazebo-exec gazebo_physics_bullet ${bullet_lflags})
  target_link_libraries(gazebo_server gazebo_physics_bullet ${bullet_lflags})
endif (INCLUDE_BULLET)

if (INCLUDE_ODE)
  target_link_libraries(gazebo-exec gazebo_physics_ode ${ODE_LIBRARIES})
  target_link_libraries(gazebo_server gazebo_physics_ode ${ODE_LIBRARIES})
endif (INCLUDE_ODE)


install (TARGETS gazebo-exec DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install (TARGETS gazebo_server DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
install (FILES ${gazeboserver_headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo COMPONENT headers)
