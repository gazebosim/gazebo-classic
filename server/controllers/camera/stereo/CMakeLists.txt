include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources Stereo_Camera.cc)
SET (headers Stereo_Camera.hh)

APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
