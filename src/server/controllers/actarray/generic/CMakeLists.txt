include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Generic_Actarray.cc)
SET (headers Generic_Actarray.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
