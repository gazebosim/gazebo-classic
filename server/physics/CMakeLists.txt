include (${gazebo_cmake_dir}/GazeboUtils.cmake)

if (INCLUDE_ODE)
  add_subdirectory(ode)
endif (INCLUDE_ODE)

if (INCLUDE_BULLET)
  add_subdirectory(bullet)
endif (INCLUDE_BULLET)

set (sources Body.cc
             PhysicsFactory.cc
             PhysicsEngine.cc
             ContactParams.cc
             Mass.cc
             Joint.cc
             Geom.cc
             Shape.cc
             BoxShape.cc
             CylinderShape.cc
             PlaneShape.cc
             SphereShape.cc
             RayShape.cc
             MultiRayShape.cc
             TrimeshShape.cc
             MapShape.cc
             #HeightmapGeom.cc
) 

set (headers BallJoint.hh
             Body.hh
             PhysicsFactory.hh
             ContactParams.hh
             Hinge2Joint.hh
             HingeJoint.hh
             Joint.hh
             PhysicsEngine.hh
             SliderJoint.hh
             UniversalJoint.hh
             Mass.hh
             Geom.hh
             BoxShape.hh
             CylinderShape.hh
             MapShape.hh
             PlaneShape.hh
             RayShape.hh
             MultiRayShape.hh
             SphereShape.hh
             TrimeshShape.hh
             #HeightmapShape.hh
)

#add_library(gazebo_physics STATIC ${sources})
APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
