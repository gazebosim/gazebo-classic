include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources IR_Array.cc)
SET (headers IR_Array.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
