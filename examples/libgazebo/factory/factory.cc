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
#include <sstream>
#include <gazebo/gazebo.h>
#include <string.h>

int main()
{
  libgazebo::Client *client = new gazebo::Client();
  libgazebo::SimulationIface *simIface = new gazebo::SimulationIface();
  libgazebo::FactoryIface *factoryIface = new gazebo::FactoryIface();

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

  /// Open the Factory interface
  try
  {
    factoryIface->Open(client, "default");
  }
  catch (std::string e)
  {
    std::cout << "Gazebo error: Unable to connect to the factory interface\n"
    << e << "\n";
    return -1;
  }

  std::string sphereBegin = "<model:physical name='sphere_model'> <xyz>0 0 5.5</xyz> <body:sphere name='sphere_body'> <geom:sphere name='sphere_geom'> <mesh>default</mesh> <size>0.5</size> <density>1.0</density> <material>Gazebo/Rocky</material></geom:sphere> </body:sphere> </model:physical>";


  for (int i=0; i<3; i++)
  {
    factoryIface->Lock(1);
    if (!strcmp((const char*)factoryIface->data->newModel,""))
    {
      std::ostringstream stream;
      stream << "<model:physical name='sphere_model_" << i << "'>";
      stream <<   "<xyz>" << i*0.5 << " 0 1</xyz>";
      stream <<   "<body:sphere name='sphere_body_" << i << "'>";
      stream <<     "<geom:sphere name='sphere_geom_" << i << "'>";
      stream <<       "<size>0.1</size>";
      stream <<       "<density>1.0</density>";
      stream <<       "<visual>";
      stream <<         "<size>0.2 0.2 0.2</size>";
      stream <<         "<material>Gazebo/Rocky</material>";
      stream <<         "<mesh>unit_sphere</mesh>";
      stream <<       "</visual>";
      stream <<     "</geom:sphere>";
      stream <<   "</body:sphere>";
      stream << "</model:physical>";

      printf("Creating[%d]\n",i);
      strcpy((char*)factoryIface->data->newModel, stream.str().c_str());
    }
    factoryIface->Unlock();
    usleep(1000000);
  }

  /*for (int i=0; i<3; i++)
  {
    factoryIface->Lock(1);
    if (!strcmp((const char*)factoryIface->data->deleteModel,""))
    {
      std::ostringstream stream;

      stream << "sphere_model_" << i;

      printf("Deleting[%d]\n",i);
      strcpy((char*)factoryIface->data->deleteModel, stream.str().c_str());
    }
    factoryIface->Unlock();
    usleep(1000000);

  }*/
  return 0;
}

