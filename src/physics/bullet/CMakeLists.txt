include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${BULLET_INCLUDE_DIRS})

LINK_DIRECTORIES(${BULLET_LIBRARY_DIRS})

SET (sources BulletPhysics.cc
             BulletCollision.cc
             BulletLink.cc
             BulletMotionState.cc
)

#             BulletJoint.cc
#             BulletSliderJoint.cc
#             BulletHingeJoint.cc
#             BulletHinge2Joint.cc
#             BulletUniversalJoint.cc
#             BulletBallJoint.cc
#             BulletScrewJoint.cc
#             BulletTrimeshShape.cc
             #BulletRayGeom.cc

add_definitions(${BULLET_CFLAGS})

gz_add_library(gazebo_physics_bullet ${sources})
#add_definitions(mDBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX)
target_link_libraries( gazebo_physics_bullet ${BULLET_LFLAGS})

INSTALL_LIBRARY(gazebo_physics_bullet)
