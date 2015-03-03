set (files
  blur.compositor
  blur.material
  CMakeLists.txt
  deferred.compositor
  deferred_post.material
  deferred_post_minilight.material
  deferred_post.program
  gazebo.material
  grid.material
  kitchen.material
  noise.compositor
  glsl120.program
  oculus.compositor
  perpixel.program
  picker.material
  ShadowCaster.material
  ShadowCaster.program
  ssao.compositor
  ssao.material
  VPL.material
  VPL.program
  shadow_caster.program
  warp.material
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/media/materials/scripts/)
