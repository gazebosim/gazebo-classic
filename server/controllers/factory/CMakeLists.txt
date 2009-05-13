include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Factory.cc)
SET (headers Factory.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
