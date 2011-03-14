include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Generic_PTZ.cc)
SET (headers Generic_PTZ.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
