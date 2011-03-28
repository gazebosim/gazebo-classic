include_directories( 
  ${XML_INCLUDE_DIRS} 
  ${PROJECT_BINARY_DIR}/src/msgs
  ${PROJECT_SOURCE_DIR}/src 
)

if (INCLUDE_ODE) 
  add_subdirectory(ode)
endif()

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
             Common.cc
             Entity.cc
             World.cc
             Model.cc
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
             Common.hh
             Entity.hh
             World.hh
             Model.hh
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
