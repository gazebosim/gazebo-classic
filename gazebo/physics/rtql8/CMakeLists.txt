include (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(/usr/local/include/dart)
INCLUDE_DIRECTORIES(/usr/local/include/eigen-3.0.5)

LINK_DIRECTORIES(/usr/local/lib)
SET(RTQL8_LIBRARIES collision dynamics geometry integration kinematics lcpsolver optimizer renderer utils yui)

set (sources
             RTQL8Physics.cc
#             ODECollision.cc
             RTQL8Link.cc
#             ODEJoint.cc
#             ODESliderJoint.cc
#             ODEHingeJoint.cc
#             ODEHinge2Joint.cc
#             ODEUniversalJoint.cc
#             ODEBallJoint.cc
#             ODETrimeshShape.cc
#             ODERayShape.cc
#             ODEMultiRayShape.cc
#             ODEHeightmapShape.cc
#             ODEScrewJoint.cc
)

SET (headers
             RTQL8Physics.hh
#             ODECollision.hh
             RTQL8Link.hh
#             ODEJoint.hh
#             ODESliderJoint.hh
#             ODEHingeJoint.hh
#             ODEHinge2Joint.hh
#             ODEUniversalJoint.hh
#             ODEBallJoint.hh
#             ODETrimeshShape.hh
#             ODERayShape.hh
#             ODEMultiRayShape.hh
#             ODEHeightmapShape.hh
#             ODESphereShape.hh
#             ODEBoxShape.hh
#             ODEPlaneShape.hh
#             ODECylinderShape.hh
             RTQL8ypes.hh
#             ODEScrewJoint.hh
)

gz_add_library(gazebo_physics_rtql8 ${sources})

target_link_libraries(gazebo_physics_rtql8 ${RTQL8_LIBRARIES})

gz_install_library(gazebo_physics_rtql8)
