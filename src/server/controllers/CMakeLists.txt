include (${gazebo_cmake_dir}/GazeboUtils.cmake)

add_subdirectory(actarray)
add_subdirectory(bumper)
add_subdirectory(camera)
add_subdirectory(gripper)
add_subdirectory(imu)
add_subdirectory(irarray)
add_subdirectory(laser)
add_subdirectory(opaque)
add_subdirectory(position2d)
add_subdirectory(ptz)
#add_subdirectory(audio)

SET (sources Controller.cc 
             ControllerFactory.cc
)

SET (headers Controller.hh
             ControllerFactory.hh
)


APPEND_TO_SERVER_HEADERS(${headers})
APPEND_TO_SERVER_SOURCES(${sources})
