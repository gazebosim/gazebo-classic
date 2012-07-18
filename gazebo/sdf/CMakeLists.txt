add_subdirectory(interface)

set (headers sdf.hh sdf.h)
set (sdfs
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
  state.sdf
  surface.sdf 
  visual.sdf 
  world.sdf)

gz_install_includes("sdf" ${headers})

INSTALL(FILES ${sdfs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/sdf/)
