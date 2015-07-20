include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${BULLET_INCLUDE_DIRS})

link_directories(${BULLET_LIBRARY_DIRS})

add_definitions(${BULLET_CFLAGS})

set (sources
  BulletBallJoint.cc
  BulletCollision.cc
  BulletFixedJoint.cc
  BulletHeightmapShape.cc
  BulletHinge2Joint.cc
  BulletHingeJoint.cc
  BulletJoint.cc
  BulletLink.cc
  BulletMesh.cc
  BulletMeshShape.cc
  BulletMotionState.cc
  BulletMultiRayShape.cc
  BulletPhysics.cc
  BulletPolylineShape.cc
  BulletRayShape.cc
  BulletScrewJoint.cc
  BulletSliderJoint.cc
  BulletSurfaceParams.cc
  BulletUniversalJoint.cc
  gzBtUniversalConstraint.cc
)

set (headers
  bullet_inc.h
  bullet_math_inc.h
  BulletBallJoint.hh
  BulletBoxShape.hh
  BulletCollision.hh
  BulletCylinderShape.hh
  BulletFixedJoint.hh
  BulletHeightmapShape.hh
  BulletHinge2Joint.hh
  BulletHingeJoint.hh
  BulletJoint.hh
  BulletLink.hh
  BulletMesh.hh
  BulletMeshShape.hh
  BulletMotionState.hh
  BulletMultiRayShape.hh
  BulletPhysics.hh
  BulletPlaneShape.hh
  BulletPolylineShape.hh
  BulletRaySensor.hh
  BulletRayShape.hh
  BulletScrewJoint.hh
  BulletSliderJoint.hh
  BulletSphereShape.hh
  BulletSurfaceParams.hh
  BulletTypes.hh
  BulletUniversalJoint.hh
  gzBtUniversalConstraint.hh
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
gz_install_includes("physics/bullet" ${headers})
