include (${gazebo_cmake_dir}/GazeboUtils.cmake)

link_directories(
  ${CCD_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

# Build in ODE by default
include_directories(SYSTEM ${CMAKE_SOURCE_DIR}/deps/opende/include)
add_subdirectory(ode)

# Add Bullet support if present
if (HAVE_BULLET)
  include_directories(${BULLET_INCLUDE_DIRS})
  link_directories(${BULLET_LIBRARY_DIRS})
  add_definitions(${BULLET_CFLAGS})
  add_subdirectory(bullet)
endif()

# Add DART support if present
if (HAVE_DART)
  add_subdirectory(dart)

  include_directories(${DART_INCLUDE_DIRS})
  link_directories(${DART_LIBRARY_DIRS})
endif()

# Add Simbody support if present
if (HAVE_SIMBODY)
  add_subdirectory(simbody)

  include_directories(SYSTEM ${Simbody_INCLUDE_DIR})
  link_directories(${Simbody_LIB_DIR})
  add_definitions(${Simbody_CFLAGS})
endif()

include_directories(${TBB_INCLUDEDIR})

# Be sure to locate this definition *after* add_subdirectories
# in order to get the proper visibility when building libs
if (WIN32)
  include_directories(${libdl_include_dir})
endif()

set (sources ${sources}
  Actor.cc
  AdiabaticAtmosphere.cc
  Atmosphere.cc
  AtmosphereFactory.cc
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
  Light.cc
  LightState.cc
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
  UserCmdManager.cc
  Wind.cc
  World.cc
  WorldState.cc
)

set (headers
  Actor.hh
  AdiabaticAtmosphere.hh
  Atmosphere.hh
  AtmosphereFactory.hh
  BallJoint.hh
  Base.hh
  BoxShape.hh
  Collision.hh
  CollisionState.hh
  Contact.hh
  ContactManager.hh
  CylinderShape.hh
  Entity.hh
  FixedJoint.hh
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
  Light.hh
  LightState.hh
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
  UserCmdManager.hh
  Wind.hh
  World.hh
  WorldState.hh)

set (physics_headers "")
foreach (hdr ${headers})
  set (physics_headers "${physics_headers}#include \"gazebo/physics/${hdr}\"\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/physics.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/physics.hh)

gz_add_library(gazebo_physics ${sources})

target_compile_definitions(gazebo_physics
  PRIVATE BUILDING_DLL_GZ_PHYSICS
)

target_link_libraries(gazebo_physics
  gazebo_common
  gazebo_util
  gazebo_ode
  gazebo_opcode
  ${Boost_LIBRARIES}
  ${IGNITION-TRANSPORT_LIBRARIES}
)

# Link in Bullet support if present
if (HAVE_BULLET)
  target_link_libraries(gazebo_physics ${BULLET_LIBRARIES})
endif()

# Link in DART support if present
if (HAVE_DART)
  target_link_libraries(gazebo_physics ${DART_LIBRARIES})
endif()

# Link in Simbody support if present
if (HAVE_SIMBODY)
  target_link_libraries(gazebo_physics ${Simbody_LIBRARIES})
endif()

if (HAVE_GDAL)
  include_directories(${GDAL_INCLUDE_DIR})
  target_link_libraries(gazebo_physics ${GDAL_LIBRARY})
endif()

if (USE_PCH)
  add_pch(gazebo_physics physics_pch.hh ${Boost_PKGCONFIG_CFLAGS} "-I${TBB_INCLUDEDIR}")
endif()

gz_install_library(gazebo_physics)
gz_install_includes("physics" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/physics.hh)

# unit tests
set (gtest_sources
  BoxShape_TEST.cc
  CylinderShape_TEST.cc
  Inertial_TEST.cc
  JointController_TEST.cc
  JointState_TEST.cc
  ModelState_TEST.cc
  Road_TEST.cc
  SphereShape_TEST.cc
)

gz_build_tests(${gtest_sources} EXTRA_LIBS gazebo_physics)

# unit tests with gazebo_test_fixture
set (gtest_fixture_sources
  Actor_TEST.cc
  Atmosphere_TEST.cc
  ContactManager_TEST.cc
  Light_TEST.cc
  LightState_TEST.cc
  Model_TEST.cc
  PhysicsEngine_TEST.cc
  PresetManager_TEST.cc
  UserCmdManager_TEST.cc
  Wind_TEST.cc
  World_TEST.cc
  WorldState_TEST.cc
)

gz_build_tests(${gtest_fixture_sources}
  EXTRA_LIBS gazebo_physics gazebo_test_fixture)
