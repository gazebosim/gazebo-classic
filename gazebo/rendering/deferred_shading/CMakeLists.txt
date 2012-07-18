include (${gazebo_cmake_dir}/GazeboUtils.cmake)

set (sources
  AmbientLight.cc
  DeferredLight.cc
  DeferredLightCP.cc
  GBufferMaterialGenerator.cc
  GBufferSchemeHandler.cc
  GeomUtils.cc
  LightMaterialGenerator.cc
  MaterialGenerator.cc
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
  MaterialGenerator.hh
  NullSchemeHandler.hh
  SSAOLogic.hh
)

gz_add_library(gazebo_rendering_deferred ${sources})

target_link_libraries(gazebo_rendering_deferred
  gazebo_common 
  ${ogre_libraries} 
  ${OPENGL_LIBRARIES} 
)
                                      
gz_install_library(gazebo_rendering_deferred)
