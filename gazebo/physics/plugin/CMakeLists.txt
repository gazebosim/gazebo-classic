include (${gazebo_cmake_dir}/GazeboUtils.cmake)

include_directories(SYSTEM)

set (sources ${sources}
  plugin/PluginBallJoint.cc
  plugin/PluginCollision.cc
  plugin/PluginGearboxJoint.cc
  plugin/PluginHeightmapShape.cc
  plugin/PluginHinge2Joint.cc
  plugin/PluginHingeJoint.cc
  plugin/PluginJoint.cc
  plugin/PluginLink.cc
  plugin/PluginMesh.cc
  plugin/PluginMeshShape.cc
  plugin/PluginMultiRayShape.cc
  plugin/PluginPhysics.cc
  plugin/PluginPolylineShape.cc
  plugin/PluginRayShape.cc
  plugin/PluginScrewJoint.cc
  plugin/PluginSliderJoint.cc
  plugin/PluginSurfaceParams.cc
  plugin/PluginUniversalJoint.cc
  PARENT_SCOPE
)

SET (headers
  ../PhysicsPlugin.h
  PluginBallJoint.hh
  PluginBoxShape.hh
  PluginCollision.hh
  PluginCylinderShape.hh
  PluginGearboxJoint.hh
  PluginHeightmapShape.hh
  PluginHinge2Joint.hh
  PluginHingeJoint.hh
  PluginJoint.hh
  PluginLink.hh
  PluginMesh.hh
  PluginMeshShape.hh
  PluginMultiRayShape.hh
  PluginPhysics.hh
  PluginPlaneShape.hh
  PluginPolylineShape.hh
  PluginRayShape.hh
  PluginScrewJoint.hh
  PluginSliderJoint.hh
  PluginSphereShape.hh
  PluginSurfaceParams.hh
  PluginTypes.hh
  PluginUniversalJoint.hh
)

set (gtest_sources
  PluginPhysics_TEST.cc
)
gz_build_tests(${gtest_sources})

# gz_add_library(gazebo_physics_plugin ${headers} ${sources})
# target_link_libraries(gazebo_physics_plugin
#   # gazebo_plugin
#   gazebo_math
#   gazebo_common
#   ${TBB_LIBRARIES}
# )

# gz_install_library(gazebo_physics_plugin)

# unit tests
# set (gtest_sources
# )
# gz_build_tests(${gtest_sources})

gz_install_includes("physics/plugin" ${headers})
