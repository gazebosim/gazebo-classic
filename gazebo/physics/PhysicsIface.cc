/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include <boost/thread/mutex.hpp>
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/gazebo_config.h"

using namespace gazebo;

std::vector<physics::WorldPtr> g_worlds;

boost::mutex g_uniqueIdMutex;
uint32_t g_uniqueId = 0;

/////////////////////////////////////////////////
bool physics::load()
{
  physics::PhysicsFactory::RegisterAll();
  return true;
}

/////////////////////////////////////////////////
bool physics::fini()
{
  remove_worlds();
  return true;
}

/////////////////////////////////////////////////
physics::WorldPtr physics::create_world(const std::string &_name)
{
  physics::WorldPtr world(new physics::World(_name));
  g_worlds.push_back(world);
  return world;
}

/////////////////////////////////////////////////
physics::WorldPtr physics::get_world(const std::string &_name)
{
  if (_name.empty())
  {
    if (g_worlds.empty())
      gzerr << "no worlds\n";
    else
      return *(g_worlds.begin());
  }
  else
  {
    for (auto const &world : g_worlds)
    {
      if (world->GetName() == _name)
        return world;
    }
  }

  gzerr << "Unable to find world by name in physics::get_world["
    << _name.c_str() << "]\n";
  gzthrow("Unable to find world by name in physics::get_world(world_name)");
}

/////////////////////////////////////////////////
void physics::load_worlds(sdf::ElementPtr _sdf)
{
  for (auto &world : g_worlds)
    world->Load(_sdf);
}

/////////////////////////////////////////////////
void physics::init_worlds()
{
  for (auto &world : g_worlds)
    world->Init();
}

/////////////////////////////////////////////////
void physics::run_worlds(unsigned int _steps)
{
  for (auto &world : g_worlds)
    world->Run(_steps);
}

/////////////////////////////////////////////////
void physics::pause_worlds(bool _pause)
{
  for (auto &world : g_worlds)
    world->SetPaused(_pause);
}

/////////////////////////////////////////////////
void physics::stop_worlds()
{
  for (auto &world : g_worlds)
    world->Stop();
}

/////////////////////////////////////////////////
void physics::load_world(WorldPtr _world, sdf::ElementPtr _sdf)
{
  _world->Load(_sdf);
}

/////////////////////////////////////////////////
void physics::init_world(WorldPtr _world)
{
  _world->Init();
}

/////////////////////////////////////////////////
void physics::run_world(WorldPtr _world, unsigned int _iterations)
{
  _world->Run(_iterations);
}

/////////////////////////////////////////////////
void physics::pause_world(WorldPtr _world, bool _pause)
{
  _world->SetPaused(_pause);
}

/////////////////////////////////////////////////
void physics::stop_world(WorldPtr _world)
{
  _world->Stop();
}

/////////////////////////////////////////////////
void physics::remove_worlds()
{
  for (auto &world : g_worlds)
  {
    world->Fini();
    world.reset();
  }

  g_worlds.clear();
}

/////////////////////////////////////////////////
bool physics::worlds_running()
{
  for (auto const &world : g_worlds)
  {
    if (world->GetRunning())
      return true;
  }

  return false;
}

/////////////////////////////////////////////////
uint32_t physics::getUniqueId()
{
  boost::mutex::scoped_lock lock(g_uniqueIdMutex);
  return ++g_uniqueId;
}
