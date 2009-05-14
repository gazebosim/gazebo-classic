include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ImuSensor.cc)
set (headers ImuSensor.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
