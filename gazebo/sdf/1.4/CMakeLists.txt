set (sdfs
  1_3.convert
  actor.sdf
  box_shape.sdf
  camera.sdf
  collision.sdf
  contact.sdf
  cylinder_shape.sdf
  forcetorque.sdf
  geometry.sdf
  gripper.sdf
  gui.sdf
  heightmap_shape.sdf
  image_shape.sdf
  imu.sdf
  inertial.sdf
  joint.sdf
  light.sdf
  link.sdf
  mesh_shape.sdf
  model.sdf
  physics.sdf
  plane_shape.sdf
  plugin.sdf
  projector.sdf
  ray.sdf
  rfidtag.sdf
  rfid.sdf
  road.sdf
  root.sdf
  scene.sdf
  sensor.sdf
  sphere_shape.sdf
  sonar.sdf
  state.sdf
  surface.sdf
  visual.sdf
  world.sdf
)

install(FILES ${sdfs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo-${GAZEBO_VERSION}/sdf/1.4)

