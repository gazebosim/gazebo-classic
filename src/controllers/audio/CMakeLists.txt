include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Audio.cc)
SET (headers Audio.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
