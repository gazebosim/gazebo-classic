include_directories( . )

set (sources Sensors.cc
             Sensor.cc
             SensorFactory.cc
             SensorManager.cc
             CameraSensor.cc
) 

set (headers Sensors.hh
             Sensor.hh
             SensorTypes.hh
             SensorFactory.hh
             SensorManager.hh
             CameraSensor.hh
)

add_library(gazebo_sensors SHARED ${sources})

target_link_libraries(gazebo_sensors gazebo_common 
                                     gazebo_rendering
                                     ${libtool_library} 
                                     ${boost_libraries}
                                     )

add_executable(gzsensor server.cc)
target_link_libraries( gzsensor gazebo_common
                                gazebo_transport
                                gazebo_sensors
                                gazebo_rendering
                                gazebo_msgs
                                pthread)

INSTALL_EXECUTABLE( gzsensor )
INSTALL_LIBRARY(gazebo_sensors)
INSTALL_INCLUDES("sensors" ${headers})
