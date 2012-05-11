include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/deps/opende/include)

set (sources ODEPhysics.cc
             ODECollision.cc
             ODEBody.cc
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
             ODEBody.hh
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
             ODESphereShape.hh
             ODEBoxShape.hh
             ODEPlaneShape.hh
             ODECylinderShape.hh
             ODETypes.hh
             ODEScrewJoint.hh
)

gz_add_library(gazebo_physics_ode ${sources})
target_link_libraries(gazebo_physics_ode gazebo_ode gazebo_opcode ${TBB_LIBRARIES})

INSTALL_LIBRARY(gazebo_physics_ode)
