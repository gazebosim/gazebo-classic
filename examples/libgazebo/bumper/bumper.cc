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
  libgazebo::BumperIface *bumperIface = new gazebo::BumperIface();

  int serverId = 0;

  /// Connect to the libgazebo server
  try
  {
    client->ConnectWait(serverId, GZ_CLIENT_ID_USER_FIRST);
  }
  catch (libgazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect\n" << e << "\n";
    return -1;
  }

  /// Open the bumper Interface
  try
  {
    bumperIface->Open(client, "bumper_iface");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the bumper interface\n" << e << "\n";
    return -1;
  }

  for (int i=0; i< 10; i++)
  {
    bumperIface->Lock(1);
    int bump_count = bumperIface->data->bumper_count;

    printf("Bump Count[%d]\n", bump_count);

    for (int c = 0; c < bump_count; c++)
    {
      printf("  State[%d]\n", bumperIface->data->bumpers[c]);
    }

    bumperIface->Unlock();

    usleep(100000);
  }


  bumperIface->Close();
  delete bumperIface;
  return 0;
}

