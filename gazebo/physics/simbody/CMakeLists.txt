include (${gazebo_cmake_dir}/GazeboUtils.cmake)

#message("includes: ${Simbody_INCLUDE_DIR}\n\n")
#message("library dirs: ${Simbody_LIB_DIR}\n\n")
#message("libraries: ${Simbody_LIBRARIES}\n\n")
#message("cflags: ${Simbody_CFLAGS}\n\n")

include_directories(SYSTEM ${Simbody_INCLUDE_DIR})

link_directories(${Simbody_LIB_DIR})

add_definitions(${Simbody_CFLAGS})

set (sources
SimbodyBallJoint.cc
SimbodyCollision.cc
SimbodyHeightmapShape.cc
SimbodyHinge2Joint.cc
SimbodyHingeJoint.cc
SimbodyJoint.cc
SimbodyLink.cc
SimbodyMesh.cc
SimbodyMeshShape.cc
SimbodyModel.cc
SimbodyMultiRayShape.cc
SimbodyPhysics.cc
SimbodyPlaneShape.cc
SimbodyPolylineShape.cc
SimbodyRayShape.cc
SimbodyScrewJoint.cc
SimbodySliderJoint.cc
SimbodyUniversalJoint.cc
)

set (headers
SimbodyBallJoint.hh
SimbodyBoxShape.hh
SimbodyCollision.hh
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

gz_add_library(gazebo_physics_simbody ${sources})
target_link_libraries(gazebo_physics_simbody ${Simbody_LIBRARIES})

gz_install_library(gazebo_physics_simbody)
gz_install_includes("physics/simbody" ${headers})
