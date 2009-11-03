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

ADD_LIBRARY(gazebo_physics_bullet STATIC ${sources})

# Removes colon separated rpath. This breaks Ubuntu-intrepid 
# (possibly gcc-4.3.2).
SET_TARGET_PROPERTIES(gazebo_physics_bullet PROPERTIES SKIP_BUILD_RPATH TRUE)

TARGET_LINK_LIBRARIES( gazebo_physics_bullet BulletCollision LinearMath BulletDynamics)
INSTALL ( TARGETS gazebo_physics_bullet DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
