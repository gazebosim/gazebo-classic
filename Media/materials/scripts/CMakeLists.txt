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
  ShadowCaster.material
  ShadowCaster.program
  ssao.compositor
  ssao.material
  youbot.material
  VPL.material
  VPL.program
)
  #pssm.material
  #shadow_receiver.program
  #shadows.material

  #shadow_caster.program
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/Media/materials/scripts/)
