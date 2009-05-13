include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Pioneer2_Gripper.cc)
SET (headers Pioneer2_Gripper.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
