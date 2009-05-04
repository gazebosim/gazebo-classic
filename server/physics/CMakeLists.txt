include (${gazebo_cmake_dir}/GazeboUtils.cmake)

ADD_SUBDIRECTORY(ode)

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
) 

APPEND_TO_SERVER_SOURCES(${sources})
