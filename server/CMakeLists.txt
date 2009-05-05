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
  ${Boost_INCLUDE_DIRS}
)

########################################
# All the library search paths
LINK_DIRECTORIES(  
  ${CMAKE_SOURCE_DIR}/libgazebo 
  ${Boost_LIBRARY_DIRS}
  ${gazeboserver_link_dirs}
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

INCLUDE_DIRECTORIES( ${gazeboserver_include_dirs} )

ADD_EXECUTABLE(gazebo ${gazeboserver_sources} main.cc)

APPEND_TO_CACHED_LIST(gazeboserver_cflags ${gazeboserver_cflags_desc} "-fPIC;-WAll")
LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")

SET_SOURCE_FILES_PROPERTIES(${gazeboserver_sources} PROPERTIES COMPILE_FLAGS ${GAZEBO_CFLAGS})
TARGET_LINK_LIBRARIES( gazebo ${gazeboserver_link_libs} ${FLTK_LIBRARIES} ${Boost_LIBRARIES} gazeboshm)

INSTALL (TARGETS gazebo DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
INSTALL (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo COMPONENT headers)

########################################
# Make the gazebo rc file
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/cmake/gazeborc.cmake ${CMAKE_CURRENT_BINARY_DIR}/.gazeborc @ONLY)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/.gazeborc DESTINATION ~/)

GAZEBOSERVER_RESET_LISTS()
