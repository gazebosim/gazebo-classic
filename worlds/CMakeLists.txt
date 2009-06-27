ADD_SUBDIRECTORY(models)

SET (files audio.world 
           bandit.world
           bsp.world
           bumper.world
           epuck_single.world
           epuck.world
           federation.world
           laser.world
           lights.world
           map.world
           openal.world
           pioneer2at.world
           pioneer2dx_camera.world
           pioneer2dx_gripper.world
           pioneer2dx.world
           simplecar.world
           simpleshapes.world
           stereocamera.world
           terrain.world
           test.world
           trimesh.world
           wizbot.world
)

INSTALL(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo/worlds/)

