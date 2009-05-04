include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ImuSensor.cc)

APPEND_TO_SERVER_SOURCES(${sources})
