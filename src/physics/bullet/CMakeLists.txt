include (${gazebo_cmake_dir}/GazeboUtils.cmake)

########################################
# Include all the search paths for headers
STRING (REPLACE " " ";" boost_include_dirs_split "${boost_include_dirs}")
STRING (REPLACE " " ";" bullet_include_dirs_split "${bullet_include_dirs}")
STRING (REPLACE " " ";" threadpool_include_dirs_split "${threadpool_include_dirs}")
STRING (REPLACE " " ";" boost_library_dirs_split "${boost_library_dirs}")
STRING (REPLACE " " ";" bullet_library_dirs_split "${bullet_library_dirs}")

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
  ${boost_include_dirs_split}
  ${freeimage_include_dir}
  ${bullet_include_dirs_split}
  ${threadpool_include_dirs_split}
  ${FLTK_INCLUDE_DIR}
)

LINK_DIRECTORIES(  
 ${CMAKE_BINARY_DIR}/libgazebo 
 ${CMAKE_BINARY_DIR}/server/gui 
 ${CMAKE_BINARY_DIR}/server/physics/bullet 
 ${boost_library_dirs_split} 
 ${bullet_library_dirs_split} 
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

set (sources ${sources} BulletScrewJoint.cc)

add_definitions(${bullet_cflags})
#add_definitions(mDBT_USE_DOUBLE_PRECISION -DBT_EULER_DEFAULT_ZYX)
add_library(gazebo_physics_bullet SHARED ${sources})
target_link_libraries( gazebo_physics_bullet ${bullet_lflags} )
install( TARGETS gazebo_physics_bullet DESTINATION ${CMAKE_INSTALL_PREFIX}/lib )
