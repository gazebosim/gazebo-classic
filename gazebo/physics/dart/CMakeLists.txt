include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${DARTCore_INCLUDE_DIRS})

link_directories(${DARTCore_LIBRARY_DIRS})

set (sources
    DARTBallJoint.cc
    DARTBoxShape.cc
    DARTCollision.cc
    DARTCylinderShape.cc
    DARTFixedJoint.cc
    DARTHeightmapShape.cc
    DARTHingeJoint.cc
    DARTHinge2Joint.cc
    DARTJoint.cc
    DARTLink.cc
    DARTMesh.cc
    DARTMeshShape.cc
    DARTModel.cc
    DARTMultiRayShape.cc
    DARTPhysics.cc
    DARTPlaneShape.cc
    DARTPolylineShape.cc
    DARTRayShape.cc
    DARTScrewJoint.cc
    DARTSliderJoint.cc
    DARTSphereShape.cc
    DARTSurfaceParams.cc
    DARTUniversalJoint.cc
)

SET (headers
    DARTBallJoint.hh
    DARTBoxShape.hh
    DARTCollision.hh
    DARTCylinderShape.hh
    DARTFixedJoint.hh
    DARTHeightmapShape.hh
    DARTHingeJoint.hh
    DARTHinge2Joint.hh
    DARTJoint.hh
    DARTLink.hh
    DARTMesh.hh
    DARTMeshShape.hh
    DARTModel.hh
    DARTMultiRayShape.hh
    DARTPhysics.hh
    DARTPlaneShape.hh
    DARTPolylineShape.hh
    DARTRayShape.hh
    DARTScrewJoint.hh
    DARTSliderJoint.hh
    DARTSphereShape.hh
    DARTSurfaceParams.hh
    DARTTypes.hh
    DARTUniversalJoint.hh
    dart_inc.h
)

set (gtest_sources
  #DARTPhysics_TEST.cc
)
gz_build_tests(${gtest_sources})

gz_add_library(gazebo_physics_dart ${headers} ${sources})

target_link_libraries(gazebo_physics_dart ${DARTCore_LIBRARIES})

gz_install_library(gazebo_physics_dart)
gz_install_includes("physics/dart" ${headers})

# unit tests
#set (gtest_sources
#  DARTJoint_TEST.cc
#)
#gz_build_tests(${gtest_sources})
