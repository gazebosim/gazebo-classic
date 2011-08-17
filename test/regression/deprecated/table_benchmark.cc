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
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <libgazebo/gz.h>

libgazebo::Client *client = NULL;
libgazebo::SimulationIface *simIface = NULL;
libgazebo::FactoryIface *factoryIface = NULL;
int laser_count = 0;

std::string test_name="Table Benchmark";
std::string data_filename = "/tmp/table_benchmark.data";

void spawn_table()
{
  std::ostringstream model;

  model << "<model:physical name='table'>";
  model << "  <xyz>0 0 0.51</xyz>";
  model << "  <rpy>0 0 0</rpy>";
  model << "  <static>false</static>";
  model << "  <body:box name='body'>";

  model << "    <geom:box name='surface'>";
  model << "      <size>1.02 1.02 0.01</size>";
  model << "      <mass>1</mass>";
  model << "      <kp>100000000.0</kp>";
  model << "      <kd>1.0</kd>";
  model << "      <bounce>0</bounce>";
  model << "      <bounceVel>0</bounceVel>";
  model << "      <slip1>0.01</slip1>";
  model << "      <slip2>0.01</slip2>";
  model << "      <visual>";
  model << "        <size>1.02 1.02 .01</size>";
  model << "        <mesh>unit_box</mesh>";
  model << "        <material>Gazebo/Rocky</material>";
  model << "      </visual>";
  model << "    </geom:box>";

  model << "    <geom:box name='leg1'>";
  model << "      <xyz>-0.5 -0.5 -0.25</xyz>";
  model << "      <size>.02 .02 0.5</size>";
  model << "      <mass>1</mass>";
  model << "      <kp>100000000.0</kp>";
  model << "      <kd>1.0</kd>";
  model << "      <bounce>0</bounce>";
  model << "      <bounceVel>0</bounceVel>";
  model << "      <slip1>0.01</slip1>";
  model << "      <slip2>0.01</slip2>";
  model << "      <visual>";
  model << "        <size>0.01 0.01 0.5</size>";
  model << "        <mesh>unit_box</mesh>";
  model << "        <material>Gazebo/Rocky</material>";
  model << "      </visual>";
  model << "    </geom:box>";

  model << "    <geom:box name='leg2'>";
  model << "      <xyz>-0.5 0.5 -0.25</xyz>";
  model << "      <size>.02 .02 0.5</size>";
  model << "      <mass>1</mass>";
  model << "      <kp>100000000.0</kp>";
  model << "      <kd>1.0</kd>";
  model << "      <bounce>0</bounce>";
  model << "      <bounceVel>0</bounceVel>";
  model << "      <slip1>0.01</slip1>";
  model << "      <slip2>0.01</slip2>";
  model << "      <visual>";
  model << "        <size>0.01 0.01 0.5</size>";
  model << "        <mesh>unit_box</mesh>";
  model << "        <material>Gazebo/Rocky</material>";
  model << "      </visual>";
  model << "    </geom:box>";

  model << "    <geom:box name='leg3'>";
  model << "      <xyz>0.5 -0.5 -0.25</xyz>";
  model << "      <size>.02 .02 0.5</size>";
  model << "      <mass>1</mass>";
  model << "      <kp>100000000.0</kp>";
  model << "      <kd>1.0</kd>";
  model << "      <bounce>0</bounce>";
  model << "      <bounceVel>0</bounceVel>";
  model << "      <slip1>0.01</slip1>";
  model << "      <slip2>0.01</slip2>";
  model << "      <visual>";
  model << "        <size>0.01 0.01 0.5</size>";
  model << "        <mesh>unit_box</mesh>";
  model << "        <material>Gazebo/Rocky</material>";
  model << "      </visual>";
  model << "    </geom:box>";

  model << "    <geom:box name='leg4'>";
  model << "      <xyz>0.5 0.5 -0.25</xyz>";
  model << "      <size>.02 .02 0.5</size>";
  model << "      <mass>1</mass>";
  model << "      <kp>100000000.0</kp>";
  model << "      <kd>1.0</kd>";
  model << "      <bounce>0</bounce>";
  model << "      <bounceVel>0</bounceVel>";
  model << "      <slip1>0.01</slip1>";
  model << "      <slip2>0.01</slip2>";
  model << "      <visual>";
  model << "        <size>0.01 0.01 0.5</size>";
  model << "        <mesh>unit_box</mesh>";
  model << "        <material>Gazebo/Rocky</material>";
  model << "      </visual>";
  model << "    </geom:box>";

  model << "  </body:box>";
  model << "</model:physical>";

  factoryIface->Lock(1);
  strcpy( (char*)factoryIface->data->newModel, model.str().c_str() );
  factoryIface->Unlock();
}

void spawn_box(double x, double y, double z=1)
{
  std::ostringstream model;

  model << "<model:physical name='box'>";
  model << "  <xyz>" << x << " " << y << " " << z << "</xyz>";
  model << "  <rpy>0 0 0</rpy>";
  model << "  <body:box name='body'>";
  model << "    <geom:box name='geom'>";
  model << "      <size>0.5 0.5 0.5</size>";
  model << "      <mass>1</mass>";
  model << "      <kp>100000000.0</kp>";
  model << "      <kd>1.0</kd>";
  model << "      <bounce>0</bounce>";
  model << "      <bounceVel>0</bounceVel>";
  model << "      <slip1>0.01</slip1>";
  model << "      <slip2>0.01</slip2>";
  model << "      <visual>";
  model << "        <size>0.5 0.5 0.5</size>";
  model << "        <mesh>unit_box</mesh>";
  model << "        <material>Gazebo/Rocky</material>";
  model << "      </visual>";
  model << "    </geom:box>";
  model << "  </body:box>";
  model << "</model:physical>";

  factoryIface->Lock(1);
  strcpy( (char*)factoryIface->data->newModel, model.str().c_str() );
  factoryIface->Unlock();
}

int main()
{
  client = new libgazebo::Client();
  simIface = new libgazebo::SimulationIface();
  factoryIface = new libgazebo::FactoryIface();

  try
  {
    client->ConnectWait(0, GZ_CLIENT_ID_USER_FIRST);
  }
  catch(std::string e)
  {
    std::cerr << "Gazebo Error: Unable to connect: " << e << "\n";
    return -1;
  }

  /// Open the sim iface
  try
  {
    simIface->Open(client, "default");
  }
  catch (std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to sim iface:" << e << "\n";
    return -1;
  }

  // Open the factory iface
  try
  {
    factoryIface->Open(client, "default");
  }
  catch( std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to the factory Iface:" 
              << e << "\n";
    return -1;
  }

  double prev_z = 5;

  spawn_table();
  usleep(100000);
  spawn_box(0, 0, prev_z);
  usleep(1000000);
  simIface->StartLogEntity("box","/tmp/box.log");
  simIface->StartLogEntity("table","/tmp/table.log");
  simIface->Unpause();

  libgazebo::Vec3 linearVel, angularVel, linearAccel, angularAccel;
  libgazebo::Pose modelPose;

  double time = simIface->data->realTime;
  while ( simIface->data->realTime - time < 2.0 )
  {
    simIface->GetState("box", modelPose, linearVel, angularVel, linearAccel, angularAccel); 

    if ( fabs(prev_z - modelPose.pos.z) >= 1e-5)
      time = simIface->data->realTime;

    prev_z = modelPose.pos.z;
  }
  simIface->StopLogEntity("box");
  simIface->StopLogEntity("table");

  factoryIface->Close();
  simIface->Close();
  client->Disconnect();
}
