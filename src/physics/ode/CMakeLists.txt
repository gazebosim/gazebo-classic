include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ODEPhysics.cc
             ODEGeom.cc
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
             ODESurfaceParams.cc
)

SET (headers ODEPhysics.hh
             ODEGeom.hh
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
             ODESurfaceParams.hh
)

add_library(gazebo_physics_ode SHARED ${sources})

LIST_TO_STRING(MY_ODE_CFLAGS "${ODE_CFLAGS}")

set_target_properties(gazebo_physics_ode PROPERTIES COMPILE_FLAGS "-fPIC ${MY_ODE_CFLAGS}")
target_link_libraries( gazebo_physics_ode ${ODE_LIBRARIES})
INSTALL_LIBRARY(gazebo_physics_ode)
