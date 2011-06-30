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

#include "common/SystemPaths.hh"
#include "common/Exception.hh"

#include "sdf/interface/sdf.h"
#include "sdf/parser/parser.hh"

#include "physics/Physics.hh"

#include "transport/Transport.hh"

#include "PhysicsServer.hh"

using namespace gazebo;

PhysicsServer::PhysicsServer()
{
  this->quit = false;

  // load the configuration options 
  try
  {
    common::SystemPaths::Instance()->Load();
  }
  catch (common::Exception e)
  {
    gzthrow("Error loading the Gazebo configuration file, check the .gazeborc file on your HOME directory \n" << e); 
  }

  transport::init();

  physics::init();
}

PhysicsServer::~PhysicsServer()
{
}

void PhysicsServer::Load( const std::string &filename )
{
  
  // Load the world file
  sdf::SDFPtr sdf(new sdf::SDF());

  sdf::initFile(filename, sdf);
  sdf::readFile(filename, sdf);

  sdf::ElementPtr worldElem = sdf->root->GetElement("world");

  while(worldElem)
  {
    physics::WorldPtr world = physics::create_world("default");

    this->worlds.push_back(world);

    //Create the world
    try
    {
      physics::load_world(world, worldElem);
    }
    catch (common::Exception e)
    {
      gzthrow("Failed to load the World\n"  << e);
    }

    worldElem = sdf->root->GetNextElement("world", worldElem);
  }
}

void PhysicsServer::Init()
{
  for (unsigned int i=0; i < this->worlds.size(); i++)
    physics::init_world(this->worlds[i]);
}

void PhysicsServer::Run()
{
  for (unsigned int i=0; i < this->worlds.size(); i++)
    physics::run_world(this->worlds[i]);

  while (!this->quit)
  {
    usleep(1000000);
  }
}

void PhysicsServer::Quit()
{
  this->quit = true;
}
