include (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE_DIRECTORIES(/usr/local/include)
INCLUDE_DIRECTORIES(/usr/local/include/rtql8)
INCLUDE_DIRECTORIES(/usr/local/include/eigen-3.1.2)

LINK_DIRECTORIES(/usr/local/lib)
SET(RTQL8_LIBRARIES collision dynamics geometry integration kinematics lcpsolver optimizer renderer utils yui)

set (sources
             RTQL8Physics.cc
             RTQL8Collision.cc
             RTQL8Link.cc
             RTQL8Joint.cc
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
             RTQL8Collision.hh
             RTQL8Link.hh
             RTQL8Joint.hh
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
