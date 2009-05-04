include (${gazebo_cmake_dir}/GazeboUtils.cmake)

ADD_SUBDIRECTORY(camera)
ADD_SUBDIRECTORY(contact)
ADD_SUBDIRECTORY(imu)
ADD_SUBDIRECTORY(ir)
ADD_SUBDIRECTORY(ray)

SET (sources Sensor.cc
             SensorFactory.cc
) 

APPEND_TO_SERVER_SOURCES(${sources})
