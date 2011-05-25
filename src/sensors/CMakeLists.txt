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

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_sensors PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

target_link_libraries(gazebo_sensors gazebo_common 
                                     gazebo_rendering
                                     ${libtool_library} 
                                     ${boost_libraries}
                                     )

INSTALL_LIBRARY(gazebo_sensors)
INSTALL_INCLUDES("sensors" ${headers})
