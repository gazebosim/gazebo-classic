include (${gazebo_cmake_dir}/GazeboUtils.cmake)

INCLUDE_DIRECTORIES(
  .
  ${CMAKE_SOURCE_DIR}/libgazebo 
  gui 
  physics 
  physics/ode
  physics/bullet
  audio_video
  controllers 
  rendering 
  sensors 
  sensors/camera
  sensors/contact
  sensors/imu
  sensors/ir
  sensors/ray
  ${LIBAVCODEC_PATH}
  ${LIBAVFORMAT_PATH}
  ${libtool_include_path}
  ${gazeboserver_include_dirs}
  ${boost_include_dirs}
  ${freeimage_include_dir}
  ${bullet_include_dir}
  ${threadpool_include_dirs_split}
  ${FLTK_INCLUDE_DIR}
)

LINK_DIRECTORIES(  
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${CMAKE_BINARY_DIR}/server/gui 
 ${CMAKE_BINARY_DIR}/server/physics/bullet 
 ${boost_library_dirs} 
 ${bullet_dynamics_dirs} 
 ${bullet_collision_dirs} 
 ${bullet_softbody_dirs} 
 ${bullet_math_dirs} 
 ${freeimage_library_dir}
 ${gazeboserver_link_dirs} 
)

SET (sources BulletPhysics.cc
             BulletGeom.cc
             BulletBody.cc
             BulletMotionState.cc
             BulletJoint.cc
             BulletSliderJoint.cc
             BulletHingeJoint.cc
             BulletHinge2Joint.cc
             BulletUniversalJoint.cc
             BulletBallJoint.cc
             BulletTrimeshShape.cc
             #BulletRayGeom.cc
)

add_definitions(${bullet_cflags})

#ADD_DEFINITIONS(-DBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX)
ADD_LIBRARY(gazebo_physics_bullet SHARED ${sources})
TARGET_LINK_LIBRARIES( gazebo_physics_bullet BulletDynamics BulletCollision LinearMath )
INSTALL( TARGETS gazebo_physics_bullet DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
