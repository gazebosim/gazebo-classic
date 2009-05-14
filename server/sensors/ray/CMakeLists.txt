include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources RaySensor.cc)
set (headers RaySensor.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
