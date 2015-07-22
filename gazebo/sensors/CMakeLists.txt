include (${gazebo_cmake_dir}/GazeboUtils.cmake)

if (WIN32)
  include_directories(${libdl_include_dir})
endif()

set (sources
  AltimeterSensor.cc
  CameraSensor.cc
  ContactSensor.cc
  DepthCameraSensor.cc
  ForceTorqueSensor.cc
  GaussianNoiseModel.cc
  GpsSensor.cc
  GpuRaySensor.cc
  ImuSensor.cc
  MagnetometerSensor.cc
  MultiCameraSensor.cc
  Noise.cc
  RaySensor.cc
  RFIDSensor.cc
  RFIDTag.cc
  SensorsIface.cc
  Sensor.cc
  SensorFactory.cc
  SensorManager.cc
  SonarSensor.cc
  WirelessReceiver.cc
  WirelessTransceiver.cc
  WirelessTransmitter.cc
)

set (headers
  AltimeterSensor.hh
  CameraSensor.hh
  ContactSensor.hh
  DepthCameraSensor.hh
  ForceTorqueSensor.hh
  GaussianNoiseModel.hh
  GpsSensor.hh
  GpuRaySensor.hh
  ImuSensor.hh
  MagnetometerSensor.hh
  MultiCameraSensor.hh
  Noise.hh
  RaySensor.hh
  RFIDSensor.hh
  RFIDTag.hh
  SensorsIface.hh
  Sensor.hh
  SensorTypes.hh
  SensorFactory.hh
  SensorManager.hh
  SonarSensor.hh
  WirelessReceiver.hh
  WirelessTransceiver.hh
  WirelessTransmitter.hh
)

set (gtest_sources
  AltimeterSensor_TEST.cc
  ForceTorqueSensor_TEST.cc
  GpsSensor_TEST.cc
  ImuSensor_TEST.cc
  MagnetometerSensor_TEST.cc
  Noise_TEST.cc
  RaySensor_TEST.cc
  Sensor_TEST.cc
  SonarSensor_TEST.cc
  WirelessReceiver_TEST.cc
  WirelessTransmitter_TEST.cc
)

gz_build_tests(${gtest_sources})

set (gtest_sources_dri
  GpuRaySensor_TEST.cc
  SensorManager_TEST.cc
)

gz_build_dri_tests(${gtest_sources_dri})

set (sensors_headers "" CACHE INTERNAL "sensors headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(sensors_headers "sensors headers"
    "#include \"gazebo/sensors/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/sensors.hh.in ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)


gz_add_library(gazebo_sensors ${sources})

target_link_libraries(gazebo_sensors
  gazebo_common
  gazebo_rendering
  gazebo_physics
  ${libtool_library}
  ${Boost_LIBRARIES}
  ${ogre_ldflags}
  )

gz_install_library(gazebo_sensors)
gz_install_includes("sensors" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/sensors.hh)
