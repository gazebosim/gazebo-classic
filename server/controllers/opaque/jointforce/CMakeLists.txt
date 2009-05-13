include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources JointForce.cc)
SET (headers JointForce.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
