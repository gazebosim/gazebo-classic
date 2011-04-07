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

#include "common/GazeboConfig.hh"
#include "common/XMLConfig.hh"
#include "common/GazeboError.hh"

#include "physics/World.hh"
#include "physics/PhysicsFactory.hh"

#include "transport/Transport.hh"

#include "PhysicsServer.hh"

using namespace gazebo;

const std::string default_config =
"<?xml version='1.0'?>\
<gazebo>\
  <world name='default'>\
    <scene>\
      <ambient>0.1 0.1 0.1 1</ambient>\
      <background_color>.1 .1 .1 1.0</background_color>\
      <shadows enabled='false' color='0.2 0.2 0.2 1.0' type='texture_modulative'/>\
      <grid>false</grid>\
    </scene>\
    <physics type='ode'>\
      <step_time>0.001</step_time>\
      <gravity>0 0 -9.8</gravity>\
      <cfm>0.0000000001</cfm>\
      <erp>0.2</erp>\
      <step_type>quick</step_type>\
      <step_iters>10</step_iters>\
      <stepW>1.3</stepW>\
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>\
      <contact_surface_layer>0.0</contact_surface_layer>\
    </physics>\
    <model name='box1'>\
      <origin xyz='0 0 1'/>\
      <link name='body'>\
        <collision name='geom'>\
          <geometry>\
            <box size='1.0 1.0 1.0'/>\
          </geometry>\
          <mass>1.0</mass>\
        </collision>\
        <visual>\
          <geometry>\
            <box size='1.0 1.0 1.0'/>\
            <!--<mesh filename='willowgarage.dae' scale='0.0254 0.0254 0.0254'/>-->\
          </geometry>\
          <mesh filename='unit_box'/>\
        </visual>\
      </link>\
    </model>\
    <model name='plane1_model'>\
      <static>true</static>\
      <link name='body'>\
        <collision name='geom'>\
          <geometry>\
            <plane normal='0 0 1'/>\
          </geometry>\
          <contact_coefficients mu1='109999.0' mu2='1000.0'/>\
        </collision>\
        <visual>\
          <geometry>\
            <plane normal='0 0 1' size='100 100' offset='0'/>\
          </geometry>\
          <material name='Gazebo/GreyGrid' uv_tile='100 100'/>\
          <cast_shadows>false</cast_shadows>\
        </visual>\
      </link>\
    </model>\
  </world>\
</gazebo>";

PhysicsServer::PhysicsServer()
{
  this->quit = false;

  // load the configuration options 
  try
  {
    common::GazeboConfig::Instance()->Load();
  }
  catch (common::GazeboError e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  transport::init();

  physics::init();
  //physics::PhysicsFactory::RegisterAll();
}

PhysicsServer::~PhysicsServer()
{
}

void PhysicsServer::Load( const std::string &filename )
{
  // Load the world file
  gazebo::common::XMLConfig *xmlFile = new gazebo::common::XMLConfig();

  try
  {
    if (!filename.empty())
      xmlFile->Load(filename);
    else
      xmlFile->LoadString(default_config);
  }
  catch (common::GazeboError e)
  {
    gzthrow("The XML config file can not be loaded, please make sure is a correct file\n" << e); 
  }

  common::XMLConfigNode *rootNode(xmlFile->GetRootNode());
  if (!rootNode || rootNode->GetName() != "gazebo")
    gzthrow("Invalid xml. Needs a root node with the <gazebo> tag");

  common::XMLConfigNode *worldNode = rootNode->GetChild("world");

  while(worldNode)
  {
    physics::create_world("default");
    //physics::World *world = new physics::World();
    this->worlds.push_back(world);

    //Create the world
    try
    {
      world->Load(worldNode);
    }
    catch (common::GazeboError e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }

    //common::XMLConfigNode *pluginNode = worldNode->GetChild("plugin");
    //while (pluginNode != NULL)
    //{
    //  this->AddPlugin( pluginNode->GetString("filename","",1), 
    //      pluginNode->GetString("handle","",1) );
    //  pluginNode = pluginNode->GetNext("plugin");
    //}

    worldNode = worldNode->GetNext("world");
  }
}

void PhysicsServer::Init()
{
  for (int i=0; i < this->worlds.size(); i++)
    this->worlds[i]->Init();
}

void PhysicsServer::Run()
{
  for (int i=0; i < this->worlds.size(); i++)
    this->worlds[i]->Start();

  while (!this->quit)
  {
    /*for (int i=0; i < this->worlds.size(); i++)
      this->worlds[i]->Update();
      */
    usleep(1000000);
  }
}

void PhysicsServer::Quit()
{
  this->quit = true;
}
