= Player Laser Interface =

This example uses a hokuyo laser and the Player laser interface.

 1. Start Gazebo
  $ gzserver laser.world

 2. Start Player
  $ player laser.cfg

 3. Start the Gazebo GUI
  $ gzclient

 4. Start playerv
  $ playerv

 5. Subscribe to the laser interface using playerv. 

 6. In the Gazebo GUI add shapes to the world, and playerv will visualize the new laser readings.
