include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
  bullet/BulletBallJoint.cc
  bullet/BulletCollision.cc
  bullet/BulletFixedJoint.cc
  bullet/BulletHeightmapShape.cc
  bullet/BulletHinge2Joint.cc
  bullet/BulletHingeJoint.cc
  bullet/BulletJoint.cc
  bullet/BulletLink.cc
  bullet/BulletMesh.cc
  bullet/BulletMeshShape.cc
  bullet/BulletMotionState.cc
  bullet/BulletMultiRayShape.cc
  bullet/BulletPhysics.cc
  bullet/BulletPolylineShape.cc
  bullet/BulletRayShape.cc
  bullet/BulletScrewJoint.cc
  bullet/BulletSliderJoint.cc
  bullet/BulletSurfaceParams.cc
  bullet/BulletUniversalJoint.cc
  bullet/gzBtUniversalConstraint.cc
  PARENT_SCOPE
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
)
gz_build_tests(${gtest_sources} EXTRA_LIBS gazebo_physics)

set (gtest_fixture_sources
  BulletPhysics_TEST.cc
)
gz_build_tests(${gtest_fixture_sources}
  EXTRA_LIBS gazebo_physics gazebo_test_fixture)

gz_install_includes("physics/bullet" ${headers})
