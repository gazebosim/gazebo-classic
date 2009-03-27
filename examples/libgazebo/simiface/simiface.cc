#include <string.h>
#include <iostream>
#include <math.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.h>

int gotCallback = 0;

void Callback()
{
  printf("Sim iface callback\n");
  gotCallback = 1;
}


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

  gazebo::Pose pose;
  bool got = false;

  for (int i=0; i< 600; i++)
  {
    printf("Iter [%d]\n",i);

    simIface->GetPose3d(name);

    got = false;
    while (!got)
    {
      //simIface->Lock(1);
      got = simIface->data->responseCount > 0;
      //simIface->Unlock();
      usleep(1000);
    }

    simIface->SetPose3d(name, pose);
  }

    /*gazebo::Pose pose;
    //pose.pos.x = i+0.1;
    pose.pos.z = .145;
    pose.yaw = M_PI/2;
    gazebo::Vec3 linearVel(0.2, 0, 0);
    gazebo::Vec3 angularVel(0, 0, 0);
    gazebo::Vec3 linearAccel(0, 0, 0);
    gazebo::Vec3 angularAccel(0, 0, 0);

    simIface->SetState(name, pose, linearVel, angularVel, 
                       linearAccel, angularAccel );
                       */

  /*usleep(1000000);

  printf("GO!\n");
  // First GO 
  simIface->Go(5*10e4, &Callback);

  // Wait until the callback is called
  while (gotCallback == 0)
    usleep(10000);

  gotCallback = 0;

  // Pause just for easy of visualization
  usleep(4000000);

  printf("GO!\n");
  // Second GO 
  simIface->Go(5*10e4, &Callback);

  // Wait until the callback is called
   while (gotCallback == 0)
    usleep(1000);
   
  // Example of resetting the simulator
  // simIface->Reset();
  //usleep(1000000);

  */
  simIface->Close();

  delete simIface;
  return 0;
}

