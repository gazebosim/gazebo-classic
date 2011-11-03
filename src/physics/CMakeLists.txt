add_subdirectory(ode)

include_directories(${tinyxml_include_dirs})
link_directories( ${tinyxml_library_dirs})
add_definitions(${tinyxml_cflags})

set (sources Base.cc
             Link.cc
             BoxShape.cc
             Contact.cc
             CylinderShape.cc
             Entity.cc
             Collision.cc
             HeightmapShape.cc
             Joint.cc
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
             TrimeshShape.cc
             World.cc
             Inertial.cc
) 

set (headers BallJoint.hh
             Base.hh
             Link.hh
             BoxShape.hh
             Contact.hh
             CylinderShape.hh
             Entity.hh
             Collision.hh
             HeightmapShape.hh
             Hinge2Joint.hh
             HingeJoint.hh
             Joint.hh
             JointFeedback.hh
             MapShape.hh
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
             TrimeshShape.hh
             UniversalJoint.hh
             World.hh
             Inertial.hh
)

if (ODE_SCREW_JOINT)
set (headers ${headers} ScrewJoint.hh)
endif (ODE_SCREW_JOINT)

set (physics_headers "" CACHE INTERNAL "physics headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(physics_headers "Physics Headers" "#include \"physics/${hdr}\"\n")
endforeach()
configure_file ( ${CMAKE_CURRENT_SOURCE_DIR}/physics.h.in ${CMAKE_CURRENT_BINARY_DIR}/physics.h )

add_library(gazebo_physics SHARED ${sources} ${sources})

target_link_libraries(gazebo_physics gazebo_common 
                                     gazebo_sdf_parser
                                     gazebo_physics_ode
                                     ${TBB_LIBRARIES} 
                                     ${libtool_library} 
                                     ${boost_libraries}
                                     ${tinyxml_libraries}
                                     )

add_executable(gzphysics server.cc)
target_link_libraries( gzphysics gazebo_common
                                 gazebo_transport
                                 gazebo_physics
                                 gazebo_sensors
                                 gazebo_sdf_interface
                                 gazebo_sdf_parser
                                 gazebo_msgs
                                 pthread)
INSTALL_EXECUTABLE( gzphysics )
INSTALL_LIBRARY(gazebo_physics)
INSTALL_INCLUDES("physics" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/physics.h)
