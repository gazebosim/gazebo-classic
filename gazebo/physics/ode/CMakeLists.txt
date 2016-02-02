include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
  ode/ODEBallJoint.cc
  ode/ODECollision.cc
  ode/ODEFixedJoint.cc
  ode/ODEGearboxJoint.cc
  ode/ODEHeightmapShape.cc
  ode/ODEHinge2Joint.cc
  ode/ODEHingeJoint.cc
  ode/ODEJoint.cc
  ode/ODELink.cc
  ode/ODEMesh.cc
  ode/ODEMeshShape.cc
  ode/ODEMultiRayShape.cc
  ode/ODEPhysics.cc
  ode/ODEPolylineShape.cc
  ode/ODERayShape.cc
  ode/ODEScrewJoint.cc
  ode/ODESliderJoint.cc
  ode/ODESurfaceParams.cc
  ode/ODEUniversalJoint.cc
  PARENT_SCOPE
)

set (headers
  ode_inc.h
  ODEBallJoint.hh
  ODEBoxShape.hh
  ODECollision.hh
  ODECylinderShape.hh
  ODEFixedJoint.hh
  ODEGearboxJoint.hh
  ODEHeightmapShape.hh
  ODEHinge2Joint.hh
  ODEHingeJoint.hh
  ODEJoint.hh
  ODELink.hh
  ODEMesh.hh
  ODEMeshShape.hh
  ODEMultiRayShape.hh
  ODEPhysics.hh
  ODEPlaneShape.hh
  ODEPolylineShape.hh
  ODERayShape.hh
  ODEScrewJoint.hh
  ODESliderJoint.hh
  ODESphereShape.hh
  ODESurfaceParams.hh
  ODETypes.hh
  ODEUniversalJoint.hh
)

set (gtest_sources
  ODEJoint_TEST.cc
  ODEPhysics_TEST.cc
)
gz_build_tests(${gtest_sources})

gz_install_includes("physics/ode" ${headers})
