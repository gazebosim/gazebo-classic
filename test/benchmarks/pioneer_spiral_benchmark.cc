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
#include <boost/lexical_cast.hpp>

libgazebo::Client *client = NULL;
libgazebo::SimulationIface *simIface = NULL;
libgazebo::FactoryIface *factoryIface = NULL;
libgazebo::Graphics3dIface *graphicsIface = NULL;

std::string test_name="Pioneer Spiral Benchmark";
std::string data_filename = "/tmp/pioneer_spiral_benchmark.data";

void spawn_robot()
{
  std::ostringstream model;

  model << "<model:physical name='pioneer'>";
  model << "  <xyz>0 0 0.15</xyz>";
  model << "  <rpy>0.0 0.0 0.0</rpy>";
  model << " <canonicalBody>chassis_body</canonicalBody>";
  model << " <body:box name='chassis_body'>";
  model << "   <xyz>0.0 0.0 0.0</xyz>";
  model << "   <rpy>0.0 0.0 0.0</rpy>";
  model << "   <geom:box name='chassis_geom'>";
  model << "     <xyz>0 0 0.0</xyz>";
  model << "     <size>0.445 0.277 0.17</size>";
  model << "     <mass>2.0</mass>";
  model << "     <mu1>1</mu1>";
  model << "     <visual>";
  model << "       <size>0.5 0.277 0.17</size>";
  model << "       <xyz>0 0 0.04</xyz>";
  model << "       <rpy>0 180 0</rpy>";
  model << "       <mesh>Pioneer2dx/chassis.mesh</mesh>";
  model << "       <material>Gazebo/Pioneer2Body</material>";
  model << "     </visual>";
  model << "     <visual>";
  model << "       <xyz>0.015 0 0.09</xyz>";
  model << "       <rpy>0 0 0</rpy>";
  model << "       <mesh>Pioneer2at/chassis_top.mesh</mesh>";
  model << "       <material>Gazebo/Black</material>";
  model << "     </visual>";
  model << "     <visual>";
  model << "       <xyz>0.21 0.0 0.068</xyz>";
  model << "       <rpy>0 0 0</rpy>";
  model << "       <size>0.12 0.29 0.034</size>";
  model << "       <mesh>Pioneer2at/sonarbank.mesh</mesh>";
  model << "       <material>Gazebo/Gold</material>";
  model << "     </visual>";
  model << "     <visual>";
  model << "       <xyz>-0.178 0.0 0.068</xyz>";
  model << "       <rpy>0 0 180</rpy>";
  model << "       <size>0.12 0.29 0.034</size>";
  model << "       <mesh>Pioneer2at/sonarbank.mesh</mesh>";
  model << "       <material>Gazebo/Gold</material>";
  model << "     </visual>";
  model << "   </geom:box>";
  model << " </body:box>";
  model << " <body:cylinder name='left_wheel'>";
  model << "   <xyz>0.1 -0.17 -0.0725</xyz>";
  model << "   <rpy>0 90 90</rpy>";
  model << "   <geom:cylinder name='left_wheel_geom'>";
  model << "     <size>0.075 0.05</size>";
  model << "     <mass>0.5</mass>";
  model << "     <mu1>0.5</mu1>";
  model << "     <visual>";
  model << "       <rpy>-90 0 0</rpy>";
  model << "       <size>0.15 0.05 0.15</size>";
  model << "       <mesh>Pioneer2dx/tire.mesh</mesh>";
  model << "       <material>Gazebo/Black</material>";
  model << "     </visual>";
  model << "     <visual>";
  model << "       <rpy>-90 0 0</rpy>";
  model << "       <size>0.088 0.05 0.088</size>";
  model << "       <mesh>Pioneer2at/wheel.mesh</mesh>";
  model << "       <material>Gazebo/Gold</material>";
  model << "     </visual>";
  model << "     <visual>";
  model << "       <rpy>0 0 0</rpy>";
  model << "       <xyz>0 0 0.015</xyz>";
  model << "       <size>0.04 0.04 0.08 </size>";
  model << "       <mesh>unit_cylinder</mesh>";
  model << "       <material>Gazebo/Black</material>";
  model << "     </visual>";
  model << "   </geom:cylinder>";
  model << " </body:cylinder>";
  model << " <body:cylinder name='right_wheel'>";
  model << "   <xyz>0.1 0.17 -0.0725</xyz>";
  model << "   <rpy>0 90 90</rpy>";
  model << "   <geom:cylinder name='right_wheel_geom'>";
  model << "     <size>0.075 0.05</size>";
  model << "     <mass>0.5</mass>";
  model << "     <mu1>0.5</mu1>";
  model << "     <visual>";
  model << "       <rpy>-90 0 0</rpy>";
  model << "       <size>0.15 0.05 0.15</size>";
  model << "       <mesh>Pioneer2dx/tire.mesh</mesh>";
  model << "       <material>Gazebo/Black</material>";
  model << "     </visual>";
  model << "     <visual>";
  model << "       <rpy>-90 0 0</rpy>";
  model << "       <size>0.088 0.05 0.088</size>";
  model << "       <mesh>Pioneer2at/wheel.mesh</mesh>";
  model << "       <material>Gazebo/Gold</material>";
  model << "     </visual>";
  model << "     <visual>";
  model << "       <rpy>0 0 0</rpy>";
  model << "       <xyz>0 0 -0.015</xyz>";
  model << "       <size>0.04 0.04 0.08</size>";
  model << "       <mesh>unit_cylinder</mesh>";
  model << "       <material>Gazebo/Black</material>";
  model << "     </visual>";
  model << "   </geom:cylinder>";
  model << " </body:cylinder>";
  model << " <body:sphere name='castor_body'>";
  model << "   <xyz>-0.200 0 -0.11</xyz>";
  model << "   <rpy>0 0 0</rpy>";
  model << "   <geom:sphere name='castor_geom'>";
  model << "     <size>0.04</size>";
  model << "     <mass>0.5</mass>";
  model << "     <mu1>0.5</mu1>";
  model << "     <visual>";
  model << "       <scale>0.04 0.04 0.04</scale>";
  model << "       <mesh>unit_sphere</mesh>";
  model << "       <material>Gazebo/Black</material>";
  model << "     </visual>";
  model << "   </geom:sphere>";
  model << " </body:sphere>";
  model << " <joint:hinge name='left_wheel_hinge'>";
  model << "   <body1>left_wheel</body1>";
  model << "   <body2>chassis_body</body2>";
  model << "   <anchor>left_wheel</anchor>";
  model << "   <anchorOffset>0 0 0.04</anchorOffset>";
  model << "   <axis>0 1 0</axis>";
  model << "   <erp>0.8</erp>";
  model << "   <cfm>10e-5</cfm>";
  model << " </joint:hinge>";
  model << " <joint:hinge name='right_wheel_hinge'>";
  model << "   <body1>right_wheel</body1>";
  model << "   <body2>chassis_body</body2>";
  model << "   <anchor>right_wheel</anchor>";
  model << "   <anchorOffset>0 0 -0.04</anchorOffset>";
  model << "   <axis>0 1 0</axis>";
  model << "   <erp>0.8</erp>";
  model << "   <cfm>10e-5</cfm>";
  model << " </joint:hinge>";
  model << " <joint:ball name='ball_joint'>";
  model << "   <body1>castor_body</body1>";
  model << "   <body2>chassis_body</body2>";
  model << "   <anchor>castor_body</anchor>";
  model << "   <erp>0.8</erp>";
  model << "   <cfm>10e-5</cfm>";
  model << " </joint:ball>";
  model << " <controller:differential_position2d name='controller1'>";
  model << "   <leftJoint>left_wheel_hinge</leftJoint>";
  model << "   <rightJoint>right_wheel_hinge</rightJoint>";
  model << "   <wheelSeparation>0.39</wheelSeparation>";
  model << "   <wheelDiameter>0.15</wheelDiameter>";
  model << "   <torque>5</torque>";
  model << "   <interface:position name='position_iface_0'/>";
  model << " </controller:differential_position2d>";
  model << "</model:physical>";

  factoryIface->Lock(1);
  strcpy( (char*)factoryIface->data->newModel, model.str().c_str() );
  factoryIface->Unlock();
}

void RunSim()
{
  simIface->StartLogEntity("pioneer", "/tmp/pioneer.log");
  simIface->Unpause();

  libgazebo::Pose pose;
  libgazebo::Vec3 lvel, avel, lacc, aacc;

  double time = simIface->data->realTime;
  while(simIface->data->realTime - time < 60)
  {
    simIface->GetState("pioneer::left_wheel",pose, lvel, avel, lacc, aacc);
    printf("Linear[%f %f %f]\n",lvel.x, lvel.y, lvel.z);
    printf("Angular[%f %f %f]\n",aacc.x, aacc.y, aacc.z);

    simIface->ApplyTorque("pioneer::left_wheel",libgazebo::Vec3(0,0,-30));
    //simIface->ApplyForce("pioneer::chassis_body",libgazebo::Vec3(5,0,0));
    usleep(2000);
  }

  simIface->StopLogEntity("pioneer");
  simIface->Pause();
  simIface->Reset();
}

int main()
{
  client = new libgazebo::Client();
  simIface = new libgazebo::SimulationIface();
  factoryIface = new libgazebo::FactoryIface();
  graphicsIface = new libgazebo::Graphics3dIface();

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

  spawn_robot();
  usleep(100000);

  /// Open the global graphics Interface
  try
  {
    graphicsIface->Open(client, "pioneer");
  }
  catch (std::string e)
  {
    std::cerr << "Gazebo error: Unable to connect to an interface\n" 
              << e << "\n";
    return -1;
  }

  graphicsIface->DrawRibbonTrail("trail");

  std::vector<std::string > step_types;
  std::vector<std::string >::iterator iter;
  step_types.push_back("robust");
  //step_types.push_back("world");
  //step_types.push_back("quick");

  /*for (iter = step_types.begin(); iter != step_types.end(); iter++)
  {
    std::string path = std::string("/home/nate/work/simpar/data/pioneer_spiral/") + *iter + "/";
    system((std::string("mkdir -p ")+path).c_str());

    FILE *out = fopen(std::string(path+"index.txt").c_str(), "w");
    fprintf(out,"# index step_time iterations\n");

    int i = 0;

    simIface->SetStepType(*iter);
    for (double step=0.01; step > 1e-5; step *= 0.5)
    {
      unsigned int iterations = 10;
      if (*iter == "world")
        iterations = 199;
      for (; iterations < 200; iterations +=20, i++)
      {
        simIface->SetStepTime(step);
        simIface->SetStepIterations(iterations);

        fprintf(out,"%d %f %d\n",i,step, iterations);
        std::cout << "Type[" << *iter << "] Step[" << step << "] Iterations[" << iterations << "]\n";
        */
        RunSim();
/*
        std::string mv_cmd = std::string("mv /tmp/pioneer.log ") + path + "pioneer_spiral_benchmark_" + *iter + "_" + boost::lexical_cast<std::string>(i) + ".data";
        system(mv_cmd.c_str());
      }
    }

    fclose(out);
  }
  */

  factoryIface->Close();
  simIface->Close();
  graphicsIface->Close();
  client->Disconnect();
}
