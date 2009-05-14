include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources MonoCameraSensor.cc
             StereoCameraSensor.cc
)

set (headers MonoCameraSensor.hh
             StereoCameraSensor.hh
)


APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
