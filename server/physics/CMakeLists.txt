include (${gazebo_cmake_dir}/GazeboUtils.cmake)

add_subdirectory(ode)
add_subdirectory(bullet)

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
             #HeightmapGeom.cc
             #MapGeom.cc
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
             #HeightmapShape.hh
             #MapShape.hh
             PlaneShape.hh
             RayShape.hh
             MultiRayShape.hh
             SphereShape.hh
             TrimeshShape.hh
)

#add_library(gazebo_physics STATIC ${sources})
APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
