if (INCLUDE_ODE) 
  add_subdirectory(ode)
endif()

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
)

add_library(gazebo_physics SHARED ${sources})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_physics PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

target_link_libraries(gazebo_physics gazebo_common 
                                     ${TBB_LIBRARIES} 
                                     ${libtool_library} 
                                     ${boost_libraries})

if (INCLUDE_ODE) 
  target_link_libraries(gazebo_physics gazebo_physics_ode)
endif()

install ( TARGETS gazebo_physics DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo/physics COMPONENT headers)
