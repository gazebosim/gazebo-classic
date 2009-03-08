#include <string.h>
#include <iostream>
#include <gazebo/gazebo.h>

int main()
{
  gazebo::Client *client = new gazebo::Client();
  gazebo::SimulationIface *simIface = new gazebo::SimulationIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    return -1;
  }

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    return -1;
  }

  //simIface->data->reset = 1;

  // Example of how to move a model (box1_model)
  char name[512] = "pioneer2dx_model1";

  for (int i=0; i< 10; i++)
  {
   
    gazebo::Pose pose;
    gazebo::Vec3 linearVel(0.1, 0, 0);
    gazebo::Vec3 angularVel(0, 0, 0);
    gazebo::Vec3 linearAccel(0, 0, 0);
    gazebo::Vec3 angularAccel(0, 0, 0);

    simIface->SetState(name, pose, linearVel, angularVel, 
                       linearAccel, angularAccel );
    usleep(9000000);
  }


  // Example of resetting the simulator
  // simIface->Reset();

  usleep(1000000);

  simIface->Close();

  delete simIface;
  return 0;
}

