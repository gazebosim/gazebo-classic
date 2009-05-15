INCLUDE (${gazebo_cmake_dir}/GazeboUtils.cmake)


########################################
# Include all the search paths for headers
INCLUDE_DIRECTORIES(
  .
  ${CMAKE_SOURCE_DIR}/libgazebo 
  gui 
  physics 
  physics/ode
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
  ${gazeboserver_include_dirs}
  ${boost_include_dirs}
  ${freeimage_include_dir}
)

LINK_DIRECTORIES(  
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${CMAKE_BINARY_DIR}/server/gui 
 ${boost_library_dirs} 
 ${freeimage_library_dir}
 ${gazeboserver_link_dirs} 
)

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
             Entity.cc
             GazeboError.cc
             GazeboMessage.cc
             Model.cc
             Angle.cc
             Param.cc
             GraphicsIfaceHandler.cc
             GuiAPI.cc
             OpenALAPI.cc
             Simulator.cc
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
             Vector2.hh
             Vector3.hh
             Vector4.hh
             World.hh
             XMLConfig.hh
             GazeboConfig.hh
             Angle.hh
             Param.hh
             GraphicsIfaceHandler.hh
             GuiAPI.hh
             OpenALAPI.hh
             Simulator.hh
)

APPEND_TO_SERVER_HEADERS(${headers})
APPEND_TO_SERVER_SOURCES(${sources})

ADD_LIBRARY(gazebo_server SHARED ${gazeboserver_sources})

ADD_EXECUTABLE(gazebo-exec main.cc)
SET_TARGET_PROPERTIES(gazebo-exec PROPERTIES OUTPUT_NAME "gazebo")

LIST_TO_STRING(GAZEBO_CFLAGS "${gazeboserver_cflags}")

SET_SOURCE_FILES_PROPERTIES(${gazeboserver_sources} PROPERTIES COMPILE_FLAGS "${GAZEBO_CFLAGS} ${CMAKE_C_FLAGS_${CMAKE_BUILD_TYPE}}")

IF (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  SET_SOURCE_FILES_PROPERTIES(${gazeboserver_sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
ENDIF (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

TARGET_LINK_LIBRARIES( gazebo-exec 
                                   ${gazeboserver_link_libs} 
                                   ${boost_libraries} 
                                   ${freeimage_library} 
                                   gazebo_server
                                   gazebo_av
                                   gazebo_gui
                                   gazebo 
)

TARGET_LINK_LIBRARIES( gazebo_server ${ODE_LIBRARIES} gazebo)

INSTALL (TARGETS gazebo-exec DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
INSTALL (TARGETS gazebo_server DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
INSTALL (FILES ${gazeboserver_headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo COMPONENT headers)

########################################
# Make the gazebo rc file
CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/cmake/gazeborc.cmake ${CMAKE_CURRENT_BINARY_DIR}/.gazeborc @ONLY)
INSTALL(FILES ${CMAKE_CURRENT_BINARY_DIR}/.gazeborc DESTINATION $ENV{HOME}/)

