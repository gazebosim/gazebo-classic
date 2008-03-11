#include <libplayerc++/playerc++.h>
#include <iostream>

int main()
{
  // We throw exceptions on creation if we fail
  try
  {
    using namespace PlayerCc;
    using namespace std;

    // Create a player client object, using the variables assigned by the
    // call to parse_args()
    PlayerClient robot(PlayerCc::PLAYER_HOSTNAME, PlayerCc::PLAYER_PORTNUM);

    // Subscribe to the simulation proxy 
    SimulationProxy sp(&robot, 0);

    // Print out some stuff
    std::cout << robot << std::endl;

    double x,y,z,roll,pitch,yaw, time;

    // Get the robot pose
    sp.GetPose3d((char*)"pioneer2dx_model1", x, y, z, roll, pitch, yaw, time);

    // Set the robot pose
    //sp.SetPose3d((char*)"pioneer2dx_model1", 2.0, 0.0, 5.0, 0, 0, 0);


    printf("Get Sim Time\n");
    /// Get the simulation time
    sp.GetProperty((char*)"world",(char*)"sim_time", &time, sizeof(double));
    printf("Simulation Time[%f]\n",time);

    /// Get the time the simulation has been pause
    sp.GetProperty((char*)"world",(char*)"pause_time", &time, sizeof(time));
    printf("Pause Time[%f]\n",time);

    /// Get the realtime
    sp.GetProperty((char*)"world",(char*)"real_time", &time, sizeof(time));
    printf("Real Time[%f]\n",time);
  }
  catch (PlayerCc::PlayerError e)
  {
    std::cerr << "Error:" << e << std::endl;
    return -1;
  }
}
