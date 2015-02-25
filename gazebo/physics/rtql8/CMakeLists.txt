include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(${RTQL8_INCLUDE_DIRS})

link_directories(${RTQL8_LIBRARY_DIRS})

set (sources
             RTQL8Physics.cc
             RTQL8Collision.cc
             RTQL8Link.cc
             RTQL8Joint.cc
             RTQL8SliderJoint.cc
             RTQL8HingeJoint.cc
             RTQL8Hinge2Joint.cc
             RTQL8UniversalJoint.cc
             RTQL8BallJoint.cc
             RTQL8TrimeshShape.cc
             RTQL8RayShape.cc
             RTQL8MultiRayShape.cc
             RTQL8HeightmapShape.cc
             RTQL8ScrewJoint.cc
             RTQL8Model.cc
)

SET (headers
             RTQL8Physics.hh
             RTQL8Collision.hh
             RTQL8Link.hh
             RTQL8Joint.hh
             RTQL8SliderJoint.hh
             RTQL8HingeJoint.hh
             RTQL8Hinge2Joint.hh
             RTQL8UniversalJoint.hh
             RTQL8BallJoint.hh
             RTQL8TrimeshShape.hh
             RTQL8RayShape.hh
             RTQL8MultiRayShape.hh
             RTQL8HeightmapShape.hh
             RTQL8SphereShape.hh
             RTQL8BoxShape.hh
             RTQL8PlaneShape.hh
             RTQL8CylinderShape.hh
             RTQL8Types.hh
             RTQL8ScrewJoint.hh
             RTQL8Model.hh
)

gz_add_library(gazebo_physics_rtql8 ${sources})

target_link_libraries(gazebo_physics_rtql8 ${RTQL8_LIBRARIES})

gz_install_library(gazebo_physics_rtql8)
