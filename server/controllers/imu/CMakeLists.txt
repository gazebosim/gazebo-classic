include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Generic_Imu.cc)
SET (headers Generic_Imu.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
