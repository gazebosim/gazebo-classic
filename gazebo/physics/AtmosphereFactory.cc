/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include "gazebo/physics/World.hh"
#include "gazebo/physics/Atmosphere.hh"
#include "gazebo/physics/AtmosphereFactory.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/util/system.hh"
#include "gazebo/gazebo_config.h"

GZ_PHYSICS_VISIBLE
void RegisterAdiabaticAtmosphere();

using namespace gazebo;
using namespace physics;


std::map<std::string, AtmosphereFactoryFn> AtmosphereFactory::models;

//////////////////////////////////////////////////
void AtmosphereFactory::RegisterAll()
{
  RegisterAdiabaticAtmosphere();
}

//////////////////////////////////////////////////
void AtmosphereFactory::RegisterAtmosphere(const std::string &_classname,
                                           AtmosphereFactoryFn _factoryfn)
{
  if (AtmosphereFactory::models.find(_classname) != models.end())
    return;
  AtmosphereFactory::models[_classname] = _factoryfn;
}

//////////////////////////////////////////////////
std::unique_ptr<Atmosphere> AtmosphereFactory::NewAtmosphere(
    const std::string &_classname,
    World &_world)
{
  std::unique_ptr<Atmosphere> result;

  std::map<std::string, AtmosphereFactoryFn>::iterator iter =
    AtmosphereFactory::models.find(_classname);
  if (iter != AtmosphereFactory::models.end())
    result = (iter->second)(_world);
  else
    gzerr << "Invalid Atmosphere Type[" << _classname << "]\n";

  return result;
}

//////////////////////////////////////////////////
bool AtmosphereFactory::IsRegistered(const std::string &_name)
{
  return (AtmosphereFactory::models.count(_name) > 0);
}
