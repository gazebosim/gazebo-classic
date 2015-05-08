include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${BULLET_INCLUDE_DIRS})

link_directories(${BULLET_LIBRARY_DIRS})

add_definitions(${BULLET_CFLAGS})

set (sources
  BulletBallJoint.cc
  BulletCollision.cc
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

# Restore the shared library generation when circular dependencyon sensors
# is solved: see bug 1516
# gz_add_library(gazebo_physics_bullet ${sources})
# target_link_libraries(gazebo_physics_bullet ${BULLET_LIBRARIES})
#gz_install_library(gazebo_physics_bullet)

add_library(gazebo_physics_bullet OBJECT ${sources})
set(physics_sensors_libs ${physics_sensors_libs} $<TARGET_OBJECTS:gazebo_physics_bullet>
    PARENT_SCOPE)
gz_install_includes("physics/bullet" ${headers})

#add_definitions(mDBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX)
