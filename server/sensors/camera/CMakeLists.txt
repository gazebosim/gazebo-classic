include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources MonoCameraSensor.cc
             StereoCameraSensor.cc
)

APPEND_TO_SERVER_SOURCES(${sources})
