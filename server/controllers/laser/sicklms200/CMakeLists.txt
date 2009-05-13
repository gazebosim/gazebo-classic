include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources SickLMS200_Laser.cc)
SET (headers SickLMS200_Laser.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
