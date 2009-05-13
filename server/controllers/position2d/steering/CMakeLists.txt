include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Steering_Position2d.cc Wheel.cc DriveWheel.cc FullWheel.cc)
SET (headers Steering_Position2d.hh Wheel.hh DriveWheel.hh FullWheel.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
