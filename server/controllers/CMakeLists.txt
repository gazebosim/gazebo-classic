include (${gazebo_cmake_dir}/GazeboUtils.cmake)

ADD_SUBDIRECTORY(actarray)
ADD_SUBDIRECTORY(bumper)
ADD_SUBDIRECTORY(camera)
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

SET (headers Controller.hh
             ControllerFactory.hh
)


APPEND_TO_SERVER_HEADERS(${headers})
APPEND_TO_SERVER_SOURCES(${sources})
