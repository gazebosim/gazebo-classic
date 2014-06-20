include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${BULLET_INCLUDE_DIRS})

link_directories(${BULLET_LIBRARY_DIRS})

add_definitions(${BULLET_CFLAGS})

set (sources
  BulletPhysics.cc
  BulletLink.cc
  BulletCollision.cc
  BulletMotionState.cc
  BulletMeshShape.cc
  BulletJoint.cc
  BulletHingeJoint.cc
  BulletSliderJoint.cc
  BulletUniversalJoint.cc
  BulletBallJoint.cc
  BulletHinge2Joint.cc
  BulletScrewJoint.cc
  BulletRayShape.cc
  BulletMultiRayShape.cc
  BulletHeightmapShape.cc
)

set (headers
  bullet_inc.h
  bullet_math_inc.h
  BulletBallJoint.hh
  BulletBoxShape.hh
  BulletCollision.hh
  BulletCylinderShape.hh
  BulletHeightmapShape.hh
  BulletHinge2Joint.hh
  BulletHingeJoint.hh
  BulletJoint.hh
  BulletLink.hh
  BulletMotionState.hh
  BulletMultiRayShape.hh
  BulletPhysics.hh
  BulletPlaneShape.hh
  BulletRaySensor.hh
  BulletRayShape.hh
  BulletScrewJoint.hh
  BulletSliderJoint.hh
  BulletSphereShape.hh
  BulletMeshShape.hh
  BulletTypes.hh
  BulletUniversalJoint.hh
)

set (gtest_sources
  BulletTypes_TEST.cc
  BulletPhysics_TEST.cc
)
gz_build_tests(${gtest_sources})

gz_add_library(gazebo_physics_bullet ${sources})
#add_definitions(mDBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX)
target_link_libraries(gazebo_physics_bullet ${BULLET_LIBRARIES})

gz_install_library(gazebo_physics_bullet)
