include (${gazebo_cmake_dir}/GazeboUtils.cmake)

#message("includes: ${Simbody_INCLUDE_DIR}\n\n")
#message("library dirs: ${Simbody_LIB_DIR}\n\n")
#message("libraries: ${Simbody_LIBRARIES}\n\n")
#message("cflags: ${Simbody_CFLAGS}\n\n")

include_directories(SYSTEM ${Simbody_INCLUDE_DIR})

link_directories(${Simbody_LIB_DIR})

add_definitions(${Simbody_CFLAGS})

set (sources
  SimbodyPhysics.cc
  SimbodyModel.cc
  SimbodyLink.cc
  SimbodyCollision.cc
  SimbodyTrimeshShape.cc
  SimbodyJoint.cc
  SimbodyHingeJoint.cc
  SimbodySliderJoint.cc
  SimbodyUniversalJoint.cc
  SimbodyBallJoint.cc
  SimbodyHinge2Joint.cc
  SimbodyScrewJoint.cc
  SimbodyRayShape.cc
  SimbodyMultiRayShape.cc
  SimbodyHeightmapShape.cc)

gz_add_library(gazebo_physics_simbody ${sources})
#add_definitions(mDBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX)
target_link_libraries(gazebo_physics_simbody ${Simbody_LIBRARIES})

gz_install_library(gazebo_physics_simbody)
