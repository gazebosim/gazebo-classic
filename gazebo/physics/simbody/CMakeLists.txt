include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources ${sources}
simbody/SimbodyBallJoint.cc
simbody/SimbodyCollision.cc
simbody/SimbodyFixedJoint.cc
simbody/SimbodyHeightmapShape.cc
simbody/SimbodyHinge2Joint.cc
simbody/SimbodyHingeJoint.cc
simbody/SimbodyJoint.cc
simbody/SimbodyLink.cc
simbody/SimbodyMesh.cc
simbody/SimbodyMeshShape.cc
simbody/SimbodyModel.cc
simbody/SimbodyMultiRayShape.cc
simbody/SimbodyPhysics.cc
simbody/SimbodyPlaneShape.cc
simbody/SimbodyPolylineShape.cc
simbody/SimbodyRayShape.cc
simbody/SimbodyScrewJoint.cc
simbody/SimbodySliderJoint.cc
simbody/SimbodyUniversalJoint.cc
PARENT_SCOPE
)

set (headers
SimbodyBallJoint.hh
SimbodyBoxShape.hh
SimbodyCollision.hh
SimbodyFixedJoint.hh
SimbodyCylinderShape.hh
SimbodyHeightmapShape.hh
SimbodyHinge2Joint.hh
SimbodyHingeJoint.hh
simbody_inc.h
SimbodyJoint.hh
SimbodyLink.hh
SimbodyMesh.hh
SimbodyMeshShape.hh
SimbodyModel.hh
SimbodyMultiRayShape.hh
SimbodyPhysics.hh
SimbodyPlaneShape.hh
SimbodyPolylineShape.hh
SimbodyRayShape.hh
SimbodyScrewJoint.hh
SimbodySliderJoint.hh
SimbodySphereShape.hh
SimbodyTypes.hh
SimbodyUniversalJoint.hh
)

gz_install_includes("physics/simbody" ${headers})
