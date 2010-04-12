
set (gazeboplugin_sources GazeboDriver.cc
             GazeboClient.cc
             GazeboInterface.cc
             GazeboTime.cc
             SimulationInterface.cc
             Position2dInterface.cc
             Position3dInterface.cc
             LaserInterface.cc
             CameraInterface.cc
             FiducialInterface.cc
             PTZInterface.cc
             OpaqueInterface.cc
             ActarrayInterface.cc
             GripperInterface.cc
             BumperInterface.cc
             IRInterface.cc
)

include_directories( 
  ${PLAYER_INCLUDE_DIRS} 
  ${CMAKE_SOURCE_DIR}/libgazebo 
  ${boost_include_dirs}
)

link_directories( ${PLAYER_LINK_DIRS} ${boost_library_dirs} gazeboshm)

add_library(gazeboplugin SHARED ${gazeboplugin_sources})
target_link_libraries(gazeboplugin ${PLAYER_LINK_LIBS} gazeboshm )

install(TARGETS gazeboplugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
