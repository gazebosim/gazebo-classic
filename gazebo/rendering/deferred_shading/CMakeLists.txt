include (${gazebo_cmake_dir}/GazeboUtils.cmake)
 
set (sources ${sources}
  deferred_shading/DeferredLight.cc
  deferred_shading/GBufferMaterialGenerator.cc
  deferred_shading/GBufferSchemeHandler.cc
  deferred_shading/GeomUtils.cc
  deferred_shading/MaterialGenerator.cc
  deferred_shading/MergeMaterialGenerator.cc
  deferred_shading/MergeSchemeHandler.cc
  deferred_shading/SSAOLogic.cc
  PARENT_SCOPE
)

# set (headers
#   AmbientLight.hh
#   DeferredLight.hh
#   DeferredLightCP.hh
#   GBufferMaterialGenerator.hh
#   GBufferSchemeHandler.hh
#   GeomUtils.hh
#   LightMaterialGenerator.hh
#   ListenerFactoryLogic.hh
#   MaterialGenerator.hh
#   MergeCP.hh
#   MergeMaterialGenerator.hh
#   MergeSchemeHandler.hh
#   NullSchemeHandler.hh
#   SSAOLogic.hh
#   TechniqueDefinitions.hh
# )
