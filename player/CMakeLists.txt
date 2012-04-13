set (sources GazeboDriver.cc
             GazeboInterface.cc
             GazeboTime.cc
             SimulationInterface.cc
             Position2dInterface.cc
             LaserInterface.cc
             CameraInterface.cc
)
             #Position3dInterface.cc
             #FiducialInterface.cc
             #PTZInterface.cc
             #OpaqueInterface.cc
             #ActarrayInterface.cc
             #GripperInterface.cc
             #BumperInterface.cc
             #IRInterface.cc
             #ImuInterface.cc

include_directories( 
  ${PLAYER_INCLUDE_DIRS} 
  ${PROJECT_SOURCE_DIR}/src 
  ${PROJECT_BINARY_DIR}/src 
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${tinyxml_include_dirs}
)

link_directories(${PLAYER_LINK_DIRS} ${boost_library_dirs})
add_library(gazebo_player SHARED ${sources})

target_link_libraries(gazebo_player ${PLAYER_LINK_LIBS}
  gazebo_common
  gazebo_transport
  gazebo_msgs
  gazebo_math
)

INSTALL_LIBRARY(gazebo_player)
#INSTALL_INCLUDES("player" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/physics.h)
