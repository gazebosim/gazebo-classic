include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources MonoCameraSensor.cc
             StereoCameraSensor.cc
)

APPEND_TO_SENSOR_SOURCES(${sources})
