add_definitions(${tinyxml_cflags})

include_directories(SYSTEM
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${tinyxml_include_dirs}
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src 
  ${PROJECT_BINARY_DIR}/src 
)

link_directories(  
  ${OGRE_LIBRARY_PATH}
  ${BOOST_LIBRARY_DIRS} 
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
add_dependencies( gazebo_gui gazebo_msgs )
add_dependencies( gazebo_rendering gazebo_msgs )
add_dependencies( gazebo_sensors gazebo_rendering )

add_executable(gzmaster master_main.cc Master.cc)
add_executable(gzserver server_main.cc Server.cc Master.cc)

target_link_libraries( gzmaster gazebo_common
                                gazebo_transport
                                gazebo_sdf_interface
                                gazebo_msgs
                                ${boost_libraries}
                                ${freeimage_library}
                                ${PROTOBUF_LIBRARY})

target_link_libraries( gzserver gazebo_common
                                gazebo_sdf_interface
                                gazebo_transport
                                gazebo_physics
                                gazebo_sensors
                                gazebo_rendering
                                gazebo_msgs
                                pthread
                                ${tinyxml_libraries}
                                )

if ("${CMAKE_BUILD_TYPE}" STREQUAL "PROFILE")
  target_link_libraries(gzserver gcov)
endif()



INSTALL_EXECUTABLE( gzmaster )
INSTALL_EXECUTABLE( gzserver )

add_library(gazebo_server SHARED Server.cc Master.cc ${sources})
target_link_libraries( gazebo_server gazebo_common
                                     gazebo_transport
                                     gazebo_physics
                                     gazebo_sensors
                                     gazebo_rendering
                                     gazebo_sdf_interface
                                     gazebo_msgs
                                     gazebo_math
                                     pthread)

add_dependencies( gazebo_server gazebo_msgs )
INSTALL_LIBRARY(gazebo_server)

set( headers Server.hh gazebo.h)
INSTALL_INCLUDES("" ${headers})

