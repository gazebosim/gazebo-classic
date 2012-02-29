include (${gazebo_cmake_dir}/GazeboUtils.cmake)
add_subdirectory(ode)

set (sources Base.cc
             Link.cc
             LinkState.cc
             BoxShape.cc
             Contact.cc
             CylinderShape.cc
             Entity.cc
             Collision.cc
             CollisionState.cc
             HeightmapShape.cc
             Joint.cc
             MapShape.cc
             Model.cc
             ModelState.cc
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
             WorldState.cc
             Inertial.cc
             SurfaceParams.cc
             State.cc
             JointState.cc
) 

set (headers BallJoint.hh
             Base.hh
             Link.hh
             LinkState.hh
             BoxShape.hh
             Contact.hh
             CylinderShape.hh
             Entity.hh
             Collision.hh
             CollisionState.hh
             HeightmapShape.hh
             Hinge2Joint.hh
             HingeJoint.hh
             Joint.hh
             JointFeedback.hh
             MapShape.hh
             Model.hh
             ModelState.hh
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
             WorldState.hh
             Inertial.hh
             ScrewJoint.hh
             SurfaceParams.hh
             State.hh
             JointState.hh
)

set (physics_headers "" CACHE INTERNAL "physics headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(physics_headers "Physics Headers" "#include \"physics/${hdr}\"\n")
endforeach()
configure_file ( ${CMAKE_CURRENT_SOURCE_DIR}/physics.h.in ${CMAKE_CURRENT_BINARY_DIR}/physics.h )

add_library(gazebo_physics SHARED ${sources} ${sources})

target_link_libraries(gazebo_physics gazebo_common 
                                     gazebo_sdf_interface
                                     gazebo_physics_ode
                                     ${TBB_LIBRARIES} 
                                     ${libtool_library} 
                                     ${boost_libraries}
                                     ${tinyxml_libraries}
                                     ${general_libraries}
                                     )

add_executable(gzphysics server.cc)
target_link_libraries(gzphysics gazebo_common
                                gazebo_transport
                                gazebo_physics
                                gazebo_sensors
                                gazebo_sdf_interface
                                gazebo_msgs
                                pthread
                                ${general_libraries}
                                )
INSTALL_EXECUTABLE( gzphysics )
INSTALL_LIBRARY(gazebo_physics)
INSTALL_INCLUDES("physics" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/physics.h)
