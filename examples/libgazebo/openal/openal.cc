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
#include <stdio.h>
#include <iostream>
#include <gazebo/gazebo.h>

libgazebo::Client *client = NULL;
libgazebo::SimulationIface *simIface = NULL;
libgazebo::AudioIface *audioIface = NULL;

int main()
{
  client = new libgazebo::Client();
  simIface = new libgazebo::SimulationIface();
  audioIface = new libgazebo::AudioIface();

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

  /// Open the OpenAL interface
  try
  {
    audioIface->Open(client, "sphere1_model::audio_iface_1");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the audio interface\n"
    << e << "\n";
    return -1;
  }

  printf("Play\n");
  /// Play the sound in the buffer
  audioIface->Lock(1);
  audioIface->data->cmd_play = 1;
  audioIface->Unlock();

  usleep(1000000);

  printf("Pause\n");
  /// Pause the sound
  audioIface->Lock(1);
  audioIface->data->cmd_pause = 1;
  audioIface->Unlock();

  usleep(1000000);

  printf("Continue\n");

  /// Play the sound in the buffer
  audioIface->Lock(1);
  audioIface->data->cmd_play = 1;
  audioIface->Unlock();

  return 0;
}


