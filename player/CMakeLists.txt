
SET (gazeboplugin_sources GazeboDriver.cc
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

INCLUDE_DIRECTORIES( 
  ${PLAYER_INCLUDE_DIRS} 
  ${CMAKE_SOURCE_DIR}/libgazebo 
  ${boost_include_dirs}
)

LINK_DIRECTORIES( ${PLAYER_LINK_DIRS} gazeboshm)

ADD_LIBRARY(gazeboplugin SHARED ${gazeboplugin_sources})
TARGET_LINK_LIBRARIES(gazeboplugin ${PLAYER_LINK_LIBS} gazeboshm )

INSTALL(TARGETS gazeboplugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
