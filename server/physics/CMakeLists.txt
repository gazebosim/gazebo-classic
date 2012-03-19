include (${gazebo_cmake_dir}/GazeboUtils.cmake)

STRING (REPLACE " " ";" parallel_quickstep_include_dirs_split "${parallel_quickstep_include_dirs}")
STRING (REPLACE " " ";" parallel_quickstep_library_dirs_split "${parallel_quickstep_library_dirs}")

INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}/libgazebo 
  ${parallel_quickstep_include_dirs_split}
)

LINK_DIRECTORIES(  
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${parallel_quickstep_library_dirs_split} 
)

add_subdirectory(ode)

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
             FixedJoint.cc
) 

if (ODE_SCREW_JOINT )
  set (headers BallJoint.hh
               Body.hh
               PhysicsFactory.hh
               SurfaceParams.hh
               Hinge2Joint.hh
               HingeJoint.hh
               Joint.hh
               PhysicsEngine.hh
               SliderJoint.hh
               ScrewJoint.hh
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
               FixedJoint.hh
  )
else (ODE_SCREW_JOINT)
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
               FixedJoint.hh
  )
endif (ODE_SCREW_JOINT)

add_library(gazebo_physics SHARED ${sources})
target_link_libraries(gazebo_physics ${LINK_LIBS} )

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_physics PROPERTIES LINK_FLAGS ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}} ${parallel_quickstep_lflags})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

install ( TARGETS gazebo_physics DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
#APPEND_TO_SERVER_SOURCES(${sources})
APPEND_TO_SERVER_HEADERS(${headers})
