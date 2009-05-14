include (${gazebo_cmake_dir}/GazeboUtils.cmake)


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

SET (headers BallJoint.hh
             Body.hh
             BoxGeom.hh
             ContactParams.hh
             CylinderGeom.hh
             Geom.hh
             Hinge2Joint.hh
             HingeJoint.hh
             Joint.hh
             PhysicsEngine.hh
             PlaneGeom.hh
             SliderJoint.hh
             SphereGeom.hh
             UniversalJoint.hh
             RayGeom.hh
             TrimeshGeom.hh
             HeightmapGeom.hh
             MapGeom.hh
             ode/ODEPhysics.hh
)

#ADD_LIBRARY(gazebo_physics STATIC ${sources})
APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
