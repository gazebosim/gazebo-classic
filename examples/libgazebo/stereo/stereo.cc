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

libgazebo::Client *client = NULL;
libgazebo::SimulationIface *simIface = NULL;
libgazebo::StereoCameraIface *stereoIface = NULL;
int saveCount = 0;

void SaveFrame()
{
  char tmp[1024];
  FILE *fp;
  
  sprintf(tmp, "frame%04d.pgm", saveCount);

  fp = fopen( tmp, "wb" );

  if (!fp)
  {
    printf( "unable to open file %s\n for writing", tmp );
    return;
  }

  fprintf( fp, "P5\n# Gazebo\n%d %d\n32768\n", stereoIface->data->width, stereoIface->data->height);

  double max = 0;
  for (unsigned int i=0; i<stereoIface->data->height*stereoIface->data->width; i++)
  {
    double v = stereoIface->data->left_depth[i];
    if (v > max)
      max = v;
  }

  printf("Max[%f]\n",max);
  for (unsigned int i = 0; i<stereoIface->data->height; i++)
  {
    for (unsigned int j =0; j<stereoIface->data->width; j++)
    {
      double v = stereoIface->data->left_depth[i*stereoIface->data->width+j];
      unsigned int value = (unsigned int)((v/max) * 32767);
      fwrite( &value, 2, 1, fp );
    }
  }

  fclose( fp );
  saveCount++;
}

int main()
{
  client = new libgazebo::Client();
  simIface = new libgazebo::SimulationIface();
  stereoIface = new libgazebo::StereoCameraIface();


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

  /// Open the Stereo interface
  try
  {
    stereoIface->Open(client, "stereo_iface_0");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the position interface\n"
    << e << "\n";
    return -1;
  }

  while (true)
  {
    stereoIface->Lock(1);
    SaveFrame();
    stereoIface->Unlock();

    usleep(100000);
  }

  return 0;
}


