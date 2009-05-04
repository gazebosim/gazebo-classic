include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Steering_Position2d.cc Wheel.cc DriveWheel.cc FullWheel.cc)

APPEND_TO_SERVER_SOURCES(${sources})
