include (${gazebo_cmake_dir}/GazeboUtils.cmake)

#include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/deps/opende/include)
#
#link_directories(
#  ${CCD_LIBRARY_DIRS}
#  ${Boost_LIBRARY_DIRS}
#)
#
#if (WIN32)
#  add_definitions(-DBUILDING_DLL_GZ_PHYSICS_ODE)
#endif()

set (sources ${sources}
  ode/ODEBallJoint.cc
  ode/ODECollision.cc
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

set (headers ${headers}
  ode/ode_inc.h
  ode/ODEBallJoint.hh
  ode/ODEBoxShape.hh
  ode/ODECollision.hh
  ode/ODECylinderShape.hh
  ode/ODEGearboxJoint.hh
  ode/ODEHeightmapShape.hh
  ode/ODEHinge2Joint.hh
  ode/ODEHingeJoint.hh
  ode/ODEJoint.hh
  ode/ODELink.hh
  ode/ODEMesh.hh
  ode/ODEMeshShape.hh
  ode/ODEMultiRayShape.hh
  ode/ODEPhysics.hh
  ode/ODEPlaneShape.hh
  ode/ODEPolylineShape.hh
  ode/ODERayShape.hh
  ode/ODEScrewJoint.hh
  ode/ODESliderJoint.hh
  ode/ODESphereShape.hh
  ode/ODESurfaceParams.hh
  ode/ODETypes.hh
  ode/ODEUniversalJoint.hh
  PARENT_SCOPE
)

set (gtest_sources ${gtest_sources}
  ode/ODEJoint_TEST.cc
  ode/ODEPhysics_TEST.cc
  PARENT_SCOPE
)

# gz_build_tests(${gtest_sources})
# 
# gz_add_library(gazebo_physics_ode ${headers} ${sources})
# target_link_libraries(gazebo_physics_ode
#   gazebo_ode
#   gazebo_opcode
#   gazebo_common
#   gazebo_math
#   ${TBB_LIBRARIES}
# )
# 
# gz_install_library(gazebo_physics_ode)
# gz_install_includes("physics/ode" ${headers})
# 
# # unit tests
# set (gtest_sources
# )
# gz_build_tests(${gtest_sources})
