/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/physics/ParameterManager.hh"

using namespace gazebo;
using namespace physics;

PhysicsProfile::PhysicsProfile()
{
}

PhysicsProfile::PhysicsProfile(const std::string &_name,
  const std::string &_physicsEngineName) :
    name(_name), physicsEngineName(_physicsEngineName)
{
}

void PhysicsProfile::SetParam(const std::string &_key,
    const boost::any &_value)
{
  paramMap[_key] = _value;
}

ParameterManager::ParameterManager(WorldPtr _world) : world(_world)
{
  this->physicsEngine = this->world->GetPhysicsEngine();

  // TODO: Predesigned physics engine profiles
  // TODO after designing SDF scheme: Get world SDF. Read in profiles
}

void ParameterManager::SaveWorldToProfile(const std::string &_profileName)
{
}

bool ParameterManager::SwitchToPhysicsProfile(const std::string &_profileName)
{
}

bool ParameterManager::SwitchToPhysicsProfile(int i)
{
}

PhysicsProfile ParameterManager::GetProfile(const std::string &_profileName)
{
}

std::vector<PhysicsProfile> ParameterManager::GetProfiles()
{
}

PhysicsProfile* ParameterManager::GetCurrentProfile()
{
  return this->currentProfile;
}
