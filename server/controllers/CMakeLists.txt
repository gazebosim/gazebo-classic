include (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE_DIRECTORIES( 
  .
  ${CMAKE_SOURCE_DIR}/libgazebo 
  ${CMAKE_SOURCE_DIR}/server 
  ${CMAKE_SOURCE_DIR}/server/audio_video 
  ${CMAKE_SOURCE_DIR}/server/physics 
  ${CMAKE_SOURCE_DIR}/server/rendering 
  ${CMAKE_SOURCE_DIR}/server/sensors
  ${CMAKE_SOURCE_DIR}/server/sensors/camera
  ${CMAKE_SOURCE_DIR}/server/sensors/contact
  ${CMAKE_SOURCE_DIR}/server/sensors/imu
  ${CMAKE_SOURCE_DIR}/server/sensors/ir
  ${CMAKE_SOURCE_DIR}/server/sensors/ray
  ${gazeboserver_include_dirs}
)

ADD_SUBDIRECTORY(actarray)
ADD_SUBDIRECTORY(bumper)
ADD_SUBDIRECTORY(camera)
ADD_SUBDIRECTORY(factory)
ADD_SUBDIRECTORY(gripper)
ADD_SUBDIRECTORY(imu)
ADD_SUBDIRECTORY(irarray)
ADD_SUBDIRECTORY(laser)
ADD_SUBDIRECTORY(opaque)
ADD_SUBDIRECTORY(position2d)
ADD_SUBDIRECTORY(ptz)

IF (INCLUDE_AV)
  ADD_SUBDIRECTORY(audio)
ENDIF (INCLUDE_AV)

SET (sources Controller.cc 
             ControllerFactory.cc
)

APPEND_TO_SERVER_SOURCES(${sources})
