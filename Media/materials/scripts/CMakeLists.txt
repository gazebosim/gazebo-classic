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
  perpixel.program
  pssm.material
  ShadowCaster.material
  shadow_caster.program
  ShadowCaster.program
  shadow_receiver.program
  shadows.material
  ssao.compositor
  ssao.material
  youbot.material
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/materials/scripts/)
