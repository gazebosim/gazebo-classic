include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/deps/opende/include)

link_directories(
  ${CCD_LIBRARY_DIRS}
)


set (sources ODEPhysics.cc
             ODECollision.cc
             ODELink.cc
             ODEJoint.cc
             ODESliderJoint.cc
             ODEHingeJoint.cc
             ODEHinge2Joint.cc
             ODEUniversalJoint.cc
             ODEBallJoint.cc
             ODEMeshShape.cc
             ODERayShape.cc
             ODEMultiRayShape.cc
             ODEHeightmapShape.cc
             ODEScrewJoint.cc
)

SET (headers
  ode_inc.h
  ODEPhysics.hh
  ODECollision.hh
  ODELink.hh
  ODEJoint.hh
  ODESliderJoint.hh
  ODEHingeJoint.hh
  ODEHinge2Joint.hh
  ODEUniversalJoint.hh
  ODEBallJoint.hh
  ODEMeshShape.hh
  ODERayShape.hh
  ODEMultiRayShape.hh
  ODEHeightmapShape.hh
  ODESphereShape.hh
  ODEBoxShape.hh
  ODEPlaneShape.hh
  ODECylinderShape.hh
  ODETypes.hh
  ODEScrewJoint.hh
)

set (gtest_sources
  ODEPhysics_TEST.cc
)
gz_build_tests(${gtest_sources})

gz_add_library(gazebo_physics_ode ${headers} ${sources})
target_link_libraries(gazebo_physics_ode
  gazebo_ode
  gazebo_opcode
  gazebo_math
  gazebo_common
  ${TBB_LIBRARIES}
)

gz_install_library(gazebo_physics_ode)

# unit tests
set (gtest_sources
  ODEJoint_TEST.cc
)
gz_build_tests(${gtest_sources})
