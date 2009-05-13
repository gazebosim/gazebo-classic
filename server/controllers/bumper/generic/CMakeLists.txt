include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Generic_Bumper.cc)
SET (headers Generic_Bumper.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
