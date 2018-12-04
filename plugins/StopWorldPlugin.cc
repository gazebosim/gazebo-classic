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
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "gazebo/common/Events.hh"
#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/World.hh"
#include "StopWorldPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(StopWorldPlugin)

/////////////////////////////////////////////
StopWorldPlugin::~StopWorldPlugin()
{
}

/////////////////////////////////////////////
void StopWorldPlugin::Load(int /*_argc*/, char ** /*_argv*/)
{
}

/////////////////////////////////////////////
void StopWorldPlugin::Init()
{
  this->worldConn = event::Events::ConnectWorldUpdateBegin(
        std::bind(&StopWorldPlugin::OnWorldUpdate, this));
}

/////////////////////////////////////////////
void StopWorldPlugin::OnWorldUpdate()
{
  this->world = physics::get_world();

  std::cout << "Stopping the world" << std::endl;

  this->worldConn = event::Events::ConnectStop(
        std::bind(&StopWorldPlugin::OnWorldStopped, this));

  this->world->Stop();
}

/////////////////////////////////////////////
void StopWorldPlugin::OnWorldStopped()
{
  GZ_ASSERT(!this->world->Running(), "The world is still running!");
  this->worldConn.reset();

  std::cout << "The world successfully stopped" << std::endl;
}
