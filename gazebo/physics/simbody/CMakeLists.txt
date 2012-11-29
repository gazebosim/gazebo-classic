include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${Simbody_INCLUDE_DIRS})

link_directories(${Simbody_LIBRARY_DIRS})

add_definitions(${Simbody_CFLAGS})

set (sources
  SimbodyPhysics.cc
  SimbodyLink.cc
  SimbodyCollision.cc
  SimbodyMotionState.cc
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
