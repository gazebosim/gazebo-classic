#include <libplayerc++/playerc++.h>
#include <iostream>

int main(int argc, char **argv)
{
  player_pose2d_t goalPose;
  goalPose.px = 4.0; //meters
  goalPose.py = 3.0; //meters
  goalPose.pa = 0.0; //radians
  
  // We throw exceptions on creation if we fail
  try
  {
    using namespace PlayerCc;
    using namespace std;

    for (int i=0; i<100; i++)
    {
      // Create a player client object, using the variables assigned by the
      // call to parse_args()
      PlayerClient *robot  = new PlayerClient(PlayerCc::PLAYER_HOSTNAME, PlayerCc::PLAYER_PORTNUM);

      // Subscribe to the simulation proxy 
      SimulationProxy *sp = new SimulationProxy(robot, 0);

      printf("%d\n",i);
      sleep(1);
    }
/*
    // Print out some stuff
    std::cout << robot << std::endl;

    double x,y,z,roll,pitch,yaw,time;

    // Get the robot pose
    sp.GetPose3d("pioneer2dx_model1", x, y, z, roll, pitch, yaw, time);

    // Set the robot pose
    sp.SetPose3d("pioneer2dx_model1", 2.0, 0.0, 5.0, 0, 0, 0);


    /// Get the simulation time
    sp.GetProperty("world","sim_time",&time, sizeof(time));

    printf("Simulation Time[%f]\n",time);

    /// Get the time the simulation has been pause
    sp.GetProperty("world","pause_time", &time, sizeof(time));
    printf("Pause Time[%f]\n",time);

    /// Get the realtime
    sp.GetProperty("world","real_time", &time, sizeof(time));
    printf("Real Time[%f]\n",time);
    */
  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }
}
