include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM
  ${CMAKE_SOURCE_DIR}/deps/opende/include)

set (sources ODEPhysics.cc
             ODECollision.cc
             ODELink.cc
             ODEJoint.cc
             ODESliderJoint.cc
             ODEHingeJoint.cc
             ODEHinge2Joint.cc
             ODEUniversalJoint.cc
             ODEBallJoint.cc
             ODETrimeshShape.cc
             ODERayShape.cc
             ODEMultiRayShape.cc
             ODEHeightmapShape.cc
             ODEScrewJoint.cc
)

SET (headers ODEPhysics.hh
             ODECollision.hh
             ODELink.hh
             ODEJoint.hh
             ODESliderJoint.hh
             ODEHingeJoint.hh
             ODEHinge2Joint.hh
             ODEUniversalJoint.hh
             ODEBallJoint.hh
             ODETrimeshShape.hh
             ODERayShape.hh
             ODEMultiRayShape.hh
             ODEHeightmapShape.hh
             ODETypes.hh
             ODEScrewJoint.hh
)

add_library(gazebo_physics_ode SHARED ${sources})

set_target_properties(gazebo_physics_ode PROPERTIES COMPILE_FLAGS "-fPIC")
target_link_libraries( gazebo_physics_ode gazebo_ode gazebo_opcode ${TBB_LIBRARIES} ${general_libraries})
INSTALL_LIBRARY(gazebo_physics_ode)
