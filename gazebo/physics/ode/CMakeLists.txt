include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/deps/opende/include)

link_directories(
  ${CCD_LIBRARY_DIRS}
)

if (WIN32)
  add_definitions(-DBUILDING_DLL_GZ_PHYSICS_ODE)
endif()

set (sources
  ODEBallJoint.cc
  ODECollision.cc
  ODEGearboxJoint.cc
  ODEHeightmapShape.cc
  ODEHinge2Joint.cc
  ODEHingeJoint.cc
  ODEJoint.cc
  ODELink.cc
  ODEMesh.cc
  ODEMeshShape.cc
  ODEMultiRayShape.cc
  ODEPhysics.cc
  ODEPolylineShape.cc
  ODERayShape.cc
  ODEScrewJoint.cc
  ODESliderJoint.cc
  ODESurfaceParams.cc
  ODEUniversalJoint.cc
)

SET (headers
  ode_inc.h
  ODEBallJoint.hh
  ODEBoxShape.hh
  ODECollision.hh
  ODECylinderShape.hh
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

gz_add_library(gazebo_physics_ode ${headers} ${sources})
target_link_libraries(gazebo_physics_ode
  gazebo_ode
  gazebo_opcode
  gazebo_math
  gazebo_common
  gazebo_physics
  ${TBB_LIBRARIES}
)

gz_install_library(gazebo_physics_ode)
gz_install_includes("physics/ode" ${headers})

# unit tests
set (gtest_sources
)
gz_build_tests(${gtest_sources})
