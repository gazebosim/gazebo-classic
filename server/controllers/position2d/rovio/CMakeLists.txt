include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Rovio_Position2d.cc)
SET (headers Rovio_Position2d.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
