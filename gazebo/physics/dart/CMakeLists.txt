include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
    dart/DARTBallJoint.cc
    dart/DARTBoxShape.cc
    dart/DARTCollision.cc
    dart/DARTCylinderShape.cc
    dart/DARTFixedJoint.cc
    dart/DARTHeightmapShape.cc
    dart/DARTHingeJoint.cc
    dart/DARTHinge2Joint.cc
    dart/DARTJoint.cc
    dart/DARTLink.cc
    dart/DARTMesh.cc
    dart/DARTMeshShape.cc
    dart/DARTModel.cc
    dart/DARTMultiRayShape.cc
    dart/DARTPhysics.cc
    dart/DARTPlaneShape.cc
    dart/DARTPolylineShape.cc
    dart/DARTRayShape.cc
    dart/DARTScrewJoint.cc
    dart/DARTSliderJoint.cc
    dart/DARTSphereShape.cc
    dart/DARTSurfaceParams.cc
    dart/DARTUniversalJoint.cc
    PARENT_SCOPE
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

gz_install_includes("physics/dart" ${headers})
