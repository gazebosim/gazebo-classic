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

#include "common/Console.hh"

#include "physics/World.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/Physics.hh"

using namespace gazebo;

std::map<std::string, physics::WorldPtr> g_worlds;

bool physics::load()
{
  physics::PhysicsFactory::RegisterAll();
  return true;
}

bool physics::fini()
{
  // Empty for now
  return true;
}

physics::WorldPtr physics::create_world(const std::string &_name)
{
  physics::WorldPtr world( new physics::World(_name) );
  g_worlds[_name] = world;
  return world;
}

physics::WorldPtr physics::get_world(const std::string &_name)
{
  physics::WorldPtr result;
  std::map<std::string, WorldPtr>::iterator iter;
  iter = g_worlds.find(_name);

  if (iter != g_worlds.end())
    result = iter->second;
  else
    gzerr << "Unable to find world[" << _name << "]\n";

  return result;
}

void physics::load_worlds(sdf::ElementPtr &_sdf)
{
  std::map<std::string, WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); iter++)
    iter->second->Load(_sdf);
}

void physics::init_worlds()
{
  std::map<std::string, WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); iter++)
    iter->second->Init();
}

void physics::run_worlds()
{
  std::map<std::string, WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); iter++)
    iter->second->Run();
}

void physics::pause_worlds(bool _pause)
{
  std::map<std::string, WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); iter++)
    iter->second->SetPaused(_pause);
}

void physics::stop_worlds()
{
  std::map<std::string, WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); iter++)
    iter->second->Stop();
}

void physics::load_world(WorldPtr world, sdf::ElementPtr &_sdf)
{
  world->Load(_sdf);
}

void physics::init_world(WorldPtr world)
{
  world->Init();
}

void physics::run_world(WorldPtr world)
{
  world->Run();
}

void physics::pause_world(WorldPtr world, bool pause)
{
  world->SetPaused(pause);
}

void physics::stop_world(WorldPtr world)
{
  world->Stop();
}
