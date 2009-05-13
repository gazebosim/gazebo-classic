include (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE_DIRECTORIES (
  .
  ${CMAKE_SOURCE_DIR}/libgazebo
  ${CMAKE_SOURCE_DIR}/server
  ${CMAKE_SOURCE_DIR}/server/controllers
  ${CMAKE_SOURCE_DIR}/server/physics
  ${CMAKE_SOURCE_DIR}/server/physics/ode
  ${CMAKE_SOURCE_DIR}/server/rendering
  ${gazeboserver_include_dirs} 
  ${boost_include_dirs}
)

LINK_DIRECTORIES(  

  ${CMAKE_SOURCE_DIR}/libgazebo
  )

ADD_SUBDIRECTORY(camera)
ADD_SUBDIRECTORY(contact)
ADD_SUBDIRECTORY(imu)
ADD_SUBDIRECTORY(ir)
ADD_SUBDIRECTORY(ray)

SET (sources Sensor.cc
             SensorFactory.cc
) 

SET (headers Sensor.hh
             SensorFactory.hh
)

ADD_LIBRARY(gazebo_sensors STATIC ${gazebosensor_sources} ${sources})
APPEND_TO_SERVER_HEADERS(${headers})
