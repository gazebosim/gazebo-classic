include_directories( 
  ${XML_INCLUDE_DIRS} 
  ${PROJECT_BINARY_DIR}/src/msgs
  ${PROJECT_SOURCE_DIR}/src 
)

if (INCLUDE_ODE)
  set (sources ode/ODEPhysics.cc
               ode/ODEGeom.cc
               ode/ODEBody.cc
               ode/ODEJoint.cc
               ode/ODESliderJoint.cc
               ode/ODEHingeJoint.cc
               ode/ODEHinge2Joint.cc
               ode/ODEUniversalJoint.cc
               ode/ODEBallJoint.cc
               ode/ODETrimeshShape.cc
               ode/ODERayShape.cc
               ode/ODEMultiRayShape.cc
               ode/ODEHeightmapShape.cc
               )

  set (headers ode/ODEPhysics.hh
               ode/ODEGeom.hh
               ode/ODEBody.hh
               ode/ODEJoint.hh
               ode/ODESliderJoint.hh
               ode/ODEHingeJoint.hh
               ode/ODEHinge2Joint.hh
               ode/ODEUniversalJoint.hh
               ode/ODEBallJoint.hh
               ode/ODETrimeshShape.hh
               ode/ODERayShape.hh
               ode/ODEMultiRayShape.hh
               ode/ODEHeightmapShape.hh
               )
  LIST_TO_STRING(MY_PHYSICS_CFLAGS "${ODE_CFLAGS}")
endif (INCLUDE_ODE)

#if (INCLUDE_BULLET) 
#  add_subdirectory(bullet)
#endif (INCLUDE_BULLET)

set (sources ${sources}
             Body.cc
             PhysicsFactory.cc
             PhysicsEngine.cc
             SurfaceParams.cc
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
             HeightmapShape.cc
             Contact.cc
) 

set (headers ${headers}
             BallJoint.hh
             Body.hh
             PhysicsFactory.hh
             SurfaceParams.hh
             Hinge2Joint.hh
             HingeJoint.hh
             Joint.hh
             PhysicsEngine.hh
             SliderJoint.hh
             UniversalJoint.hh
             Mass.hh
             Geom.hh
             Shape.hh
             BoxShape.hh
             CylinderShape.hh
             MapShape.hh
             PlaneShape.hh
             RayShape.hh
             MultiRayShape.hh
             SphereShape.hh
             TrimeshShape.hh
             HeightmapShape.hh
             Contact.hh
             JointFeedback.hh
)

add_library(gazebo_physics SHARED ${sources})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_physics PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

set_target_properties(gazebo_physics PROPERTIES COMPILE_FLAGS "-fPIC ${MY_PHYSICS_CFLAGS}")

target_link_libraries(gazebo_physics ${CMAKE_LINK_LIBS_${CMAKE_BUILD_TYPE}})

install ( TARGETS gazebo_physics DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
install (FILES ${headers} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/gazebo/physics COMPONENT headers)
