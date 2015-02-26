set (sdfs
  1_0.convert
  actor.sdf 
  camera.sdf
  collision.sdf 
  contact.sdf
  gazebo.sdf 
  geometry.sdf 
  gripper.sdf
  gui.sdf
  inertial.sdf 
  joint.sdf 
  light.sdf 
  link.sdf 
  model.sdf 
  physics.sdf 
  plugin.sdf 
  projector.sdf
  ray.sdf
  rfidtag.sdf
  rfid.sdf
  road.sdf
  scene.sdf 
  sensor.sdf 
  sky.sdf
  state.sdf
  surface.sdf 
  visual.sdf 
  world.sdf
)

install(FILES ${sdfs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/sdf/1.2)
