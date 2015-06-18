include (${gazebo_cmake_dir}/GazeboUtils.cmake)
add_subdirectory(ode)

if (HAVE_BULLET)
  add_subdirectory(bullet)
endif()
if (HAVE_DART)
  add_subdirectory(dart)
endif()

if (HAVE_SIMBODY)
  add_subdirectory(simbody)
endif()

if (HAVE_GDAL)
  include_directories(${GDAL_INCLUDE_DIR})
endif()

include_directories(${TBB_INCLUDEDIR})

# Be sure to locate this definition *after* add_subdirectories
# in order to get the proper visibility when building libs
if (WIN32)
  add_definitions(-DBUILDING_DLL_GZ_PHYSICS)
  include_directories(${libdl_include_dir})
endif()


set (sources
  Actor.cc
  Base.cc
  BoxShape.cc
  Collision.cc
  CollisionState.cc
  Contact.cc
  ContactManager.cc
  CylinderShape.cc
  Entity.cc
  Gripper.cc
  HeightmapShape.cc
  Inertial.cc
  Joint.cc
  JointController.cc
  JointState.cc
  Link.cc
  LinkState.cc
  MapShape.cc
  MeshShape.cc
  Model.cc
  ModelState.cc
  MultiRayShape.cc
  PhysicsIface.cc
  PhysicsEngine.cc
  PhysicsFactory.cc
  PlaneShape.cc
  PolylineShape.cc
  Population.cc
  PresetManager.cc
  RayShape.cc
  Road.cc
  Shape.cc
  SphereShape.cc
  State.cc
  SurfaceParams.cc
  World.cc
  WorldState.cc
)

set (headers
  Actor.hh
  BallJoint.hh
  Base.hh
  BoxShape.hh
  Collision.hh
  CollisionState.hh
  Contact.hh
  ContactManager.hh
  CylinderShape.hh
  Entity.hh
  HeightmapShape.hh
  Hinge2Joint.hh
  HingeJoint.hh
  GearboxJoint.hh
  Inertial.hh
  Gripper.hh
  Joint.hh
  JointController.hh
  JointWrench.hh
  JointState.hh
  Link.hh
  LinkState.hh
  MapShape.hh
  MeshShape.hh
  Model.hh
  ModelState.hh
  MultiRayShape.hh
  PhysicsIface.hh
  PhysicsEngine.hh
  PhysicsFactory.hh
  PhysicsTypes.hh
  PlaneShape.hh
  PolylineShape.hh
  Population.hh
  PresetManager.hh
  RayShape.hh
  Road.hh
  Shape.hh
  ScrewJoint.hh
  SliderJoint.hh
  SphereShape.hh
  State.hh
  SurfaceParams.hh
  UniversalJoint.hh
  World.hh
  WorldState.hh)

set (physics_headers "" CACHE INTERNAL "physics headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(physics_headers
    "Physics Headers" "#include \"gazebo/physics/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/physics.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/physics.hh)

gz_add_library(gazebo_physics ${sources})

target_link_libraries(gazebo_physics
  gazebo_common
  gazebo_util
  gazebo_physics_ode
)

if (HAVE_BULLET)
  target_link_libraries(gazebo_physics gazebo_physics_bullet)
endif()

if (HAVE_DART)
  target_link_libraries(gazebo_physics gazebo_physics_dart)
endif()

if (HAVE_SIMBODY)
  target_link_libraries(gazebo_physics gazebo_physics_simbody)
endif()

if (HAVE_GDAL)
  target_link_libraries(gazebo_physics ${GDAL_LIBRARY})
endif()

gz_install_library(gazebo_physics)
gz_install_includes("physics" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/physics.hh)

# unit tests
set (gtest_sources
  BoxShape_TEST.cc
  ContactManager_TEST.cc
  CylinderShape_TEST.cc
  Inertial_TEST.cc
  JointController_TEST.cc
  PhysicsEngine_TEST.cc
  PresetManager_TEST.cc
  Road_TEST.cc
  SphereShape_TEST.cc
)
gz_build_tests(${gtest_sources})
