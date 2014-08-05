set (sources GazeboDriver.cc
             GazeboInterface.cc
             GazeboTime.cc
             SimulationInterface.cc
             Position2dInterface.cc
             LaserInterface.cc
             CameraInterface.cc
             ../../gazebo/gazebo.cc
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
  ${SDFormat_INCLUDE_DIRS} 
  ${PLAYER_INCLUDE_DIRS} 
  ${OPENGL_INCLUDE_DIR}
  ${OGRE_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

link_directories( 
  ${SDFormat_LIBRARY_DIRS} 
)

link_directories(${PLAYER_LINK_DIRS} ${Boost_LIBRARY_DIRS})

gz_add_library(gazebo_player ${sources})
target_link_libraries(gazebo_player ${PLAYER_LINK_LIBS}
gazebo_player
gazebo_common
gazebo_transport
gazebo_msgs
gazebo_math
gazebo_util
)

gz_install_library(gazebo_player)
#gz_install_includes("player" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/physics.h)
