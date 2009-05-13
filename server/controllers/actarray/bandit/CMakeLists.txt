include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Bandit_Actarray.cc)
SET (headers Bandit_Actarray.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
