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

#include "common/XMLConfig.hh"

#include "physics/World.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/Physics.hh"

using namespace gazebo;

bool physics::init()
{
  physics::PhysicsFactory::RegisterAll();
  return true;
}

physics::WorldPtr physics::create_world(const std::string &name)
{
  physics::WorldPtr world( new physics::World(name) );
  return world;
}

void physics::load_world(WorldPtr world, common::XMLConfigNode *node)
{
  world->Load(node);
}

void physics::init_world(WorldPtr world)
{
  world->Init();
}

void physics::run_world(WorldPtr world)
{
    world->Start();
}

void physics::pause_world(WorldPtr world, bool pause)
{
  world->SetPaused(pause);
}

void physics::stop_world(WorldPtr world)
{
  world->Stop();
}
