add_definitions(${tinyxml_cflags})

include_directories( 
  ${PROJECT_SOURCE_DIR}/src 
  ${PROJECT_BINARY_DIR}/src 
  ${ODE_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${tinyxml_include_dirs}
)

link_directories(  
  ${OGRE_LIBRARY_PATH}
  ${BOOST_LIBRARY_DIRS} 
  ${assimp_library_dirs} 
  ${ODE_LIBRARY_DIRS}
  ${tinyxml_library_dirs}
)

add_subdirectory(msgs)
add_subdirectory(math)
add_subdirectory(sdf)
add_subdirectory(transport)
add_subdirectory(common)
add_subdirectory(rendering)
add_subdirectory(gui)
add_subdirectory(physics)
add_subdirectory(sensors)

add_dependencies( gazebo_physics gazebo_msgs )
add_dependencies( gazebo_physics_ode gazebo_msgs )
#add_dependencies( gazebo_gui gazebo_msgs )
add_dependencies( gazebo_rendering gazebo_msgs )
add_dependencies( gazebo_sensors gazebo_rendering )

add_executable(gzphysics physics_server.cc PhysicsServer.cc)
add_executable(gzmaster master.cc Master.cc)
add_executable(gzsensor sensor_server.cc SensorServer.cc)
add_executable(gzserver server.cc Server.cc Master.cc)

target_link_libraries( gzmaster gazebo_common
                                gazebo_transport
                                gazebo_sdf_interface
                                gazebo_msgs
                                ${boost_libraries}
                                ${freeimage_library}
                                ${assimp_libraries}
                                ${PROTOBUF_LIBRARY})

target_link_libraries( gzserver gazebo_common
                                gazebo_sdf_interface
                                gazebo_sdf_parser
                                gazebo_transport
                                gazebo_physics
                                gazebo_sensors
                                gazebo_rendering
                                gazebo_msgs
                                assimp
                                pthread
                                ${tinyxml_libraries})

target_link_libraries( gzphysics gazebo_common
                                 gazebo_transport
                                 gazebo_physics
                                 gazebo_sdf_interface
                                 gazebo_sdf_parser
                                 gazebo_msgs
                                 assimp
                                 pthread)

target_link_libraries( gzsensor gazebo_common
                                gazebo_transport
                                gazebo_sensors
                                gazebo_rendering
                                gazebo_msgs
                                pthread)

INSTALL_EXECUTABLE( gzmaster )
INSTALL_EXECUTABLE( gzphysics )
INSTALL_EXECUTABLE( gzsensor )
INSTALL_EXECUTABLE( gzserver )

add_library(gazebo_server SHARED Server.cc Master.cc ${sources})
target_link_libraries( gazebo_server gazebo_common
                                     gazebo_transport
                                     gazebo_physics
                                     gazebo_sensors
                                     gazebo_rendering
                                     gazebo_msgs
                                     assimp
                                     pthread
                                     gazebo_sdf_parser)
add_dependencies( gazebo_server gazebo_msgs )
INSTALL_LIBRARY(gazebo_server)

set( headers Server.hh gazebo.h)
INSTALL_INCLUDES("" ${headers})

