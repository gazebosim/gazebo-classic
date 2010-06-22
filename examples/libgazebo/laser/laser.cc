#include <string.h>
#include <iostream>
#include <math.h>
#include <boost/bind.hpp>
#include <gazebo/gazebo.h>

int main()
{
  libgazebo::Client *client = new gazebo::Client();
  libgazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  libgazebo::LaserIface *laserIface = new gazebo::LaserIface();

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

  /// Open the Simulation Interface
  try
  {
    laserIface->Open(client, "laser_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the laser interface\n" << e << "\n";
    return -1;
  }

  while (true)
  {
    float maxRange = 0;
    for (int i = 0; i < laserIface->data->range_count; i++)
    {
      if (laserIface->data->ranges[i] > maxRange)
        maxRange = laserIface->data->ranges[i];
    }
    printf("Max Range[%f]\n",maxRange);
    usleep(10000);
  }

  laserIface->Close();
  simIface->Close();

  delete laserIface;
  delete simIface;
  return 0;
}

