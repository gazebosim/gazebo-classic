include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Generic_Camera.cc)
SET (headers Generic_Camera.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
