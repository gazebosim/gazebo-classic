include (${gazebo_cmake_dir}/GazeboUtils.cmake)

SET (sources BulletPhysics.cc
             BulletGeom.cc
             BulletBody.cc
             BulletMotionState.cc
             BulletJoint.cc
             BulletSliderJoint.cc
             BulletHingeJoint.cc
             BulletHinge2Joint.cc
             BulletUniversalJoint.cc
             BulletBallJoint.cc
             #BulletRayGeom.cc
)

#ADD_DEFINITIONS(-DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX)
ADD_LIBRARY(gazebo_physics_bullet SHARED ${sources})
TARGET_LINK_LIBRARIES( gazebo_physics_bullet BulletDynamics BulletCollision LinearMath )
INSTALL( TARGETS gazebo_physics_bullet DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
