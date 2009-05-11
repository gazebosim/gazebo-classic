INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)

########################################
# Include all the search paths for headers
INCLUDE_DIRECTORIES( .  
  ${CMAKE_SOURCE_DIR}/libgazebo 
  audio_video
  gui 
  rendering 
  physics 
  physics/ode
  sensors 
  sensors/camera
  sensors/contact
  sensors/imu
  sensors/ir
  sensors/ray
  controllers 
  ${gazeboserver_include_dirs}
  ${LIBAVCODEC_PATH}
  ${LIBAVFORMAT_PATH}
  ${FLTK_INCLUDE_DIR}
  ${boost_include_dirs}
)

########################################
# All the library search paths
LINK_DIRECTORIES(  
 ${CMAKE_SOURCE_DIR}/libgazebo ${boost_library_dirs} ${gazeboserver_link_dirs} 
)

########################################
# Process all the subdirectories
ADD_SUBDIRECTORY(rendering)
ADD_SUBDIRECTORY(sensors)
ADD_SUBDIRECTORY(physics)
ADD_SUBDIRECTORY(controllers)
ADD_SUBDIRECTORY(gui)
IF (INCLUDE_AV)
  ADD_SUBDIRECTORY(audio_video)
ENDIF (INCLUDE_AV)

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
             GazeboError.cc
             GazeboMessage.cc
             Model.cc
             Simulator.cc
             Angle.cc
             Param.cc
             GraphicsIfaceHandler.cc
)

SET (headers Entity.hh
           GazeboError.hh
           GazeboMessage.hh
           Global.hh
           Model.hh
           Pose3d.hh
           Quatern.hh
           Simulator.hh
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
)

APPEND_TO_SERVER_SOURCES(${sources})

ADD_EXECUTABLE(gazebo ${gazeboserver_sources} main.cc)

LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")

SET_SOURCE_FILES_PROPERTIES(${gazeboserver_sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")

IF (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  SET_SOURCE_FILES_PROPERTIES(${gazeboserver_sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
ENDIF (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

TARGET_LINK_LIBRARIES( gazebo ${gazeboserver_link_libs} ${FLTK_LIBRARIES} ${boost_libraries} gazeboshm )

INSTALL (TARGETS gazebo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
INSTALL (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo COMPONENT headers)

########################################
# Make the gazebo rc file
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/cmake/gazeborc.cmake ${CMAKE_CURRENT_BINARY_DIR}/.gazeborc @ONLY)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/.gazeborc DESTINATION ~/)

