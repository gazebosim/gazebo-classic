include (${gazebo_cmake_dir}/GazeboUtils.cmake)

STRING (REPLACE " " ";" parallel_quickstep_include_dirs_split "${parallel_quickstep_include_dirs}")
STRING (REPLACE " " ";" parallel_quickstep_library_dirs_split "${parallel_quickstep_library_dirs}")

include_directories(
  ${parallel_quickstep_include_dirs_split}
)

link_directories(
 ${parallel_quickstep_library_dirs_split} 
)


SET (sources ODEPhysics.cc
             ODEGeom.cc
             ODEBody.cc
             ODEJoint.cc
             ODESliderJoint.cc
             ODEHingeJoint.cc
             ODEHinge2Joint.cc
             ODEUniversalJoint.cc
             ODEBallJoint.cc
             ODETrimeshShape.cc
             ODERayShape.cc
             ODEMultiRayShape.cc
             ODEHeightmapShape.cc
)

SET (headers ODEPhysics.hh
             ODEGeom.hh
             ODEBody.hh
             ODEJoint.hh
             ODESliderJoint.hh
             ODEHingeJoint.hh
             ODEHinge2Joint.hh
             ODEUniversalJoint.hh
             ODEBallJoint.hh
             ODETrimeshShape.hh
             ODERayShape.hh
             ODEMultiRayShape.hh
             ODEHeightmapShape.hh
)

add_library(gazebo_physics_ode SHARED ${sources})

if (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})
  set_source_files_properties(${sources} PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
  set_target_properties(gazebo_physics_ode PROPERTIES LINK_FLAGS 
    ${CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE}})
endif (CMAKE_LINK_FLAGS_${CMAKE_BUILD_TYPE})

LIST_TO_STRING(MY_ODE_CFLAGS "${ODE_CFLAGS}")

add_definitions(${parallel_quickstep_cflags})
set_target_properties( gazebo_physics_ode PROPERTIES COMPILE_FLAGS "-fPIC ${MY_ODE_CFLAGS}" )
target_link_libraries( gazebo_physics_ode ${ODE_LIBRARIES} ${parallel_quickstep_lflags} )
install ( TARGETS gazebo_physics_ode DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
