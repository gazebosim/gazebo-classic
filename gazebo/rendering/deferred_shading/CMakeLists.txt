include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources
  DeferredLight.cc
  GBufferMaterialGenerator.cc
  GBufferSchemeHandler.cc
  GeomUtils.cc
  MaterialGenerator.cc
  MergeMaterialGenerator.cc
  MergeSchemeHandler.cc
  SSAOLogic.cc
)

set (headers
  AmbientLight.hh
  DeferredLight.hh
  DeferredLightCP.hh
  GBufferMaterialGenerator.hh
  GBufferSchemeHandler.hh
  GeomUtils.hh
  LightMaterialGenerator.hh
  ListenerFactoryLogic.hh
  MaterialGenerator.hh
  MergeCP.hh
  MergeMaterialGenerator.hh
  MergeSchemeHandler.hh
  NullSchemeHandler.hh
  SSAOLogic.hh
  TechniqueDefinitions.hh
)

link_directories(  
  ${ogre_library_dirs}
)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBUILDING_DLL_GZ_RENDERING_DEFERRED")

gz_add_library(gazebo_rendering_deferred ${sources})

target_link_libraries(gazebo_rendering_deferred
  gazebo_common 
  ${ogre_libraries} 
  ${OPENGL_LIBRARIES} 
)
                                      
gz_install_library(gazebo_rendering_deferred)
