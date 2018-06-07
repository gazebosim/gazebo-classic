/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/common/Events.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "DistributedSimulationManager.hh"

#include "DistributedSimulationPrivate.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(DistributedSimulationManager)

/////////////////////////////////////////////
DistributedSimulationManager::~DistributedSimulationManager()
{
}

/////////////////////////////////////////////
void DistributedSimulationManager::Load(int /*_argc*/, char ** /*_argv*/)
{
}

/////////////////////////////////////////////
void DistributedSimulationManager::Init()
{
  this->dataPtr = std::make_unique<DistributedSimulationPrivate>();
  this->dataPtr->node = std::make_unique<ignition::transport::Node>();
}
