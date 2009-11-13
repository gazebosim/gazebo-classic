include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources ODEPhysics.cc
             ODEGeom.cc
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
)

add_library(gazebo_physics_ode SHARED ${sources})
target_link_libraries( gazebo_physics_ode ${ODE_LIBRARIES})
install ( TARGETS gazebo_physics_ode DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
