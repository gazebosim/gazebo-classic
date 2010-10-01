include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources MonoCameraSensor.cc)

set (headers MonoCameraSensor.hh)


APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
