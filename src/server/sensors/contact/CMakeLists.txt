include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ContactSensor.cc)
set (headers ContactSensor.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
