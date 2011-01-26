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

////////////////////////////////////////////////////////////////////////////////
// Save an image frame
void SaveFrame(const char *filename, libgazebo::CameraData *data)
{
  int i, width, height;
  FILE *file;

  file = fopen(filename, "w+");
  if (!file)
    return;

  width = data->width;
  height = data->height;

  int pixelSize = 3;
  int rowSize = width * pixelSize;

  // Write ppm
  fprintf(file, "P6\n%d %d\n%d\n", width, height, 255);
  for (i = 0; i < height; i++)
    fwrite(data->image + i * rowSize, rowSize, 1, file);

  fclose(file);
}

int main()
{
  libgazebo::Client *client = new gazebo::Client();
  libgazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  libgazebo::CameraIface *camIface = new gazebo::CameraIface();

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

  /// Open the Simulation Interface
  try
  {
    simIface->Open(client, "default");
  }
  catch (libgazebo::GazeboError e)
  {
    std::cout << "Gazebo error: Unable to connect to the sim interface\n" << e << "\n";
    return -1;
  }

  /// Open the Camera interface
  try
  {
    camIface->Open(client, "camera_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the camera interface\n"
    << e << "\n";
    return -1;
  }

  // Save frames using gazebo's build in method
  printf("Save frames on\n");
  camIface->Lock(1);
  camIface->data->saveFrames = true;
  camIface->Unlock();
  usleep(1000000);

  printf("Save frames off\n");
  camIface->Lock(1);
  camIface->data->saveFrames = false;
  camIface->Unlock();

  // Manually save frames
  int count = 0;
  char filename[50];
  while (true)
  {
    sprintf(filename,"save_%d.pnm",count);
    camIface->Lock(1);
    SaveFrame(filename, camIface->data);
    camIface->Unlock();
    usleep(100000);
    count++;
  }

  return 0;
}

