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
#include <gazebo/gazebo.h>
#include <gazebo/GazeboError.hh>

int main()
{
  libgazebo::Client *client = new gazebo::Client();
  libgazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  libgazebo::PositionIface *posIface = new gazebo::PositionIface();

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

  /// Open the Position interface
  try
  {
    posIface->Open(client, "position_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the position interface\n"
    << e << "\n";
    return -1;
  }

  // Enable the motor
  posIface->Lock(1);
  posIface->data->cmdEnableMotors = 1;
  posIface->Unlock();

  posIface->data->cmdVelocity.yaw = -0.1;
  while (true)
  {
    posIface->Lock(1);
    posIface->data->cmdVelocity.pos.x = 0.2;
    posIface->Unlock();

    usleep(100000);
  }
  return 0;
}

