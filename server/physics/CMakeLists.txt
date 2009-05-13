include (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE_DIRECTORIES(. 
  ode 
  ${CMAKE_SOURCE_DIR}/libgazebo 
  ${CMAKE_SOURCE_DIR}/server 
  ${CMAKE_SOURCE_DIR}/server/rendering 
  ${CMAKE_SOURCE_DIR}/server/sensors 
  ${gazeboserver_include_dirs} 
  ${boost_include_dirs}
)

SET (sources BallJoint.cc
             Body.cc
             BoxGeom.cc
             ContactParams.cc
             CylinderGeom.cc
             Geom.cc
             Hinge2Joint.cc
             HingeJoint.cc
             Joint.cc
             PhysicsEngine.cc
             PlaneGeom.cc
             SliderJoint.cc
             SphereGeom.cc
             UniversalJoint.cc
             RayGeom.cc
             TrimeshGeom.cc
             HeightmapGeom.cc
             MapGeom.cc
             ode/ODEPhysics.cc
) 

ADD_LIBRARY(gazebo_physics STATIC ${sources})
