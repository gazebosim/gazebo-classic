include (${gazebo_cmake_dir}/GazeboUtils.cmake)

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

set_target_properties(gazebo_physics_ode PROPERTIES COMPILE_FLAGS "-fPIC ${MY_ODE_CFLAGS}")
target_link_libraries( gazebo_physics_ode ${ODE_LIBRARIES})
install ( TARGETS gazebo_physics_ode DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
