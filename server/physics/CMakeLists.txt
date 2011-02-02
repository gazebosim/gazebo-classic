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

set (headers BallJoint.hh
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

target_link_libraries(gazebo_physics)

install ( TARGETS gazebo_physics DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
#APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
