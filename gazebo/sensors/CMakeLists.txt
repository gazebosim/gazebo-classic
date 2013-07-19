include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources 
  CameraSensor.cc
  ContactSensor.cc
  DepthCameraSensor.cc
  GpsSensor.cc
  GpuRaySensor.cc
  ImuSensor.cc
  MultiCameraSensor.cc
  Noise.cc
  RaySensor.cc
  RFIDSensor.cc
  RFIDTag.cc
  Sensors.cc
  Sensor.cc
  SensorFactory.cc
  SensorManager.cc
) 

set (headers
  CameraSensor.hh
  ContactSensor.hh
  DepthCameraSensor.hh
  GpsSensor.hh
  GpuRaySensor.hh
  ImuSensor.hh
  MultiCameraSensor.hh
  Noise.hh
  RaySensor.hh
  RFIDSensor.hh
  RFIDTag.hh
  Sensors.hh
  Sensor.hh
  SensorTypes.hh
  SensorFactory.hh
  SensorManager.hh
)

set (gtest_sources
  RaySensor_TEST.cc
  ImuSensor_TEST.cc
  Sensor_TEST.cc
)

gz_build_tests(${gtest_sources})

set (gtest_sources_dri
  SensorManager_TEST.cc
  GpuRaySensor_TEST.cc
)

gz_build_dri_tests(${gtest_sources_dri})

set (sensors_headers "" CACHE INTERNAL "sensors headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(sensors_headers "sensors headers"
    "#include \"gazebo/sensors/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/sensors.hh.in ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)


add_library(gazebo_sensors SHARED ${sources})

target_link_libraries(gazebo_sensors gazebo_common 
                                     gazebo_rendering
                                     gazebo_physics
                                     ${libtool_library} 
                                     ${Boost_LIBRARIES}
                                     ${ogre_ldflags}
                                     )

add_executable(gzsensor server.cc ../gazebo.cc)
target_link_libraries( gzsensor gazebo_common
                                gazebo_transport
                                gazebo_sensors
                                gazebo_rendering
                                gazebo_msgs
                                pthread)

gz_install_executable( gzsensor )
gz_install_library(gazebo_sensors)
gz_install_includes("sensors" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)
