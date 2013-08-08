include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${DART_INCLUDEDIR}
                    ${FCL_INCLUDEDIR}
                    ${ASSIMP_INCLUDEDIR})

link_directories(${DART_LIBRARY_DIRS}
                 ${FCL_LIBRARY_DIRS}
                 ${ASSIMP_LIBRARY_DIRS})

add_definitions(${DART_DEFINITIONS})

set (sources
    DARTPhysics.cc
    DARTCollision.cc
    DARTLink.cc
    DARTJoint.cc
    DARTSliderJoint.cc
    DARTHingeJoint.cc
    DARTHinge2Joint.cc
    DARTUniversalJoint.cc
    DARTBallJoint.cc
    DARTMeshShape.cc
    DARTRayShape.cc
    DARTMultiRayShape.cc
    DARTHeightmapShape.cc
    DARTScrewJoint.cc
    DARTModel.cc
    DARTUtils.cc
)

SET (headers
    DART_inc.h
    DARTPhysics.hh
    DARTCollision.hh
    DARTLink.hh
    DARTJoint.hh
    DARTSliderJoint.hh
    DARTHingeJoint.hh
    DARTHinge2Joint.hh
    DARTUniversalJoint.hh
    DARTBallJoint.hh
    DARTMeshShape.hh
    DARTRayShape.hh
    DARTMultiRayShape.hh
    DARTHeightmapShape.hh
    DARTSphereShape.hh
    DARTBoxShape.hh
    DARTPlaneShape.hh
    DARTCylinderShape.hh
    DARTTypes.hh
    DARTScrewJoint.hh
    DARTModel.hh
    DARTUtils.hh
)

set (gtest_sources
  #DARTPhysics_TEST.cc
)
gz_build_tests(${gtest_sources})

gz_add_library(gazebo_physics_dart ${sources})

# these flags should probably go into dart.pc
target_link_libraries(gazebo_physics_dart ${DART_LIBRARIES} -lfcl -lassimp)

gz_install_library(gazebo_physics_dart)

# unit tests
set (gtest_sources
  DARTJoint_TEST.cc
)
gz_build_tests(${gtest_sources})
