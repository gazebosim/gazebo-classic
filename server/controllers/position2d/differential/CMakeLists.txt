include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Differential_Position2d.cc)
SET (headers Differential_Position2d.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
