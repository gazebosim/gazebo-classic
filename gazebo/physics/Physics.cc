/*
 * Copyright 2012 Open Source Robotics Foundation
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
#include "common/Exception.hh"
#include "physics/World.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/Physics.hh"
#include "gazebo_config.h"

using namespace gazebo;

std::vector<physics::WorldPtr> g_worlds;

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
    for (std::vector<WorldPtr>::iterator iter = g_worlds.begin();
        iter != g_worlds.end(); ++iter)
    {
      if ((*iter)->GetName() == _name)
        return (*iter);
    }
  }

  gzerr << "Unable to find world by name in physics::get_world["
    << _name.c_str() << "]\n";
  gzthrow("Unable to find world by name in physics::get_world(world_name)");
}

/////////////////////////////////////////////////
void physics::load_worlds(sdf::ElementPtr _sdf)
{
  std::vector<WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); ++iter)
    (*iter)->Load(_sdf);
}

/////////////////////////////////////////////////
void physics::init_worlds()
{
  std::vector<WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); ++iter)
  {
    gzerr << "init_worlds iter " << *iter << '\n';
    (*iter)->Init();
  }
}

/////////////////////////////////////////////////
void physics::run_worlds()
{
  std::vector<WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); ++iter)
    (*iter)->Run();
}

/////////////////////////////////////////////////
void physics::pause_worlds(bool _pause)
{
  std::vector<WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); ++iter)
    (*iter)->SetPaused(_pause);
}

/////////////////////////////////////////////////
void physics::stop_worlds()
{
  std::vector<WorldPtr>::iterator iter;
  for (iter = g_worlds.begin(); iter != g_worlds.end(); ++iter)
    (*iter)->Stop();
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
void physics::run_world(WorldPtr _world)
{
  _world->Run();
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
  for (std::vector<WorldPtr>::iterator iter = g_worlds.begin();
      iter != g_worlds.end(); ++iter)
  {
    (*iter)->Fini();
    (*iter).reset();
  }

  g_worlds.clear();
}


