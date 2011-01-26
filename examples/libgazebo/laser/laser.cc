/*
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
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

