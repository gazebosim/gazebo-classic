if (INCLUDE_ODE) 
  add_subdirectory(ode)
endif()

include_directories(${tinyxml_include_dirs})
link_directories( ${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

set (sources Base.cc
             Body.cc
             BoxShape.cc
             Contact.cc
             CylinderShape.cc
             Entity.cc
             Geom.cc
             HeightmapShape.cc
             Joint.cc
             Mass.cc
             MapShape.cc
             Model.cc
             MultiRayShape.cc
             Physics.cc
             PhysicsEngine.cc
             PhysicsFactory.cc
             PlaneShape.cc
             RayShape.cc
             Shape.cc
             SphereShape.cc
             SurfaceParams.cc
             TrimeshShape.cc
             World.cc
             Inertial.cc
) 

set (headers BallJoint.hh
             Base.hh
             Body.hh
             BoxShape.hh
             Contact.hh
             CylinderShape.hh
             Entity.hh
             Geom.hh
             HeightmapShape.hh
             Hinge2Joint.hh
             HingeJoint.hh
             Joint.hh
             JointFeedback.hh
             MapShape.hh
             Mass.hh
             Model.hh
             MultiRayShape.hh
             Physics.hh
             PhysicsEngine.hh
             PhysicsFactory.hh
             PhysicsTypes.hh
             PlaneShape.hh
             RayShape.hh
             Shape.hh
             SliderJoint.hh
             SphereShape.hh
             SurfaceParams.hh
             TrimeshShape.hh
             UniversalJoint.hh
             World.hh
             Inertial.hh
)

add_library(gazebo_physics SHARED ${sources} ${sources})

target_link_libraries(gazebo_physics gazebo_common 
                                     gazebo_sdf_parser
                                     ${TBB_LIBRARIES} 
                                     ${libtool_library} 
                                     ${boost_libraries}
                                     ${tinyxml_libraries})

if (INCLUDE_ODE) 
  target_link_libraries(gazebo_physics gazebo_physics_ode)
endif()

INSTALL_LIBRARY(gazebo_physics)
INSTALL_INCLUDES("physics" ${headers})
