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

#include <gazebo/physics/World.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include "gazebo/util/Joystick.hh"
#include "DroneDemoWorldPlugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(DroneDemoWorldPlugin)

/////////////////////////////////////////////////
DroneDemoWorldPlugin::DroneDemoWorldPlugin()
{
  this->joy = new util::Joystick();
}

/////////////////////////////////////////////////
void DroneDemoWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
{
  this->world = _world;
  this->joy->Init(0);

  this->timeLimit = common::Time(180);

  this->node = transport::NodePtr(new transport::Node());
  this->worldControlPub =
      this->node->Advertise<msgs::WorldControl>("~/world_control");
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&DroneDemoWorldPlugin::OnUpdate, this));             }

/////////////////////////////////////////////////
void DroneDemoWorldPlugin::Reset()
{
  this->timer.Stop();
  this->timer.Reset();
}

/////////////////////////////////////////////////
void DroneDemoWorldPlugin::Init()
{
}

/////////////////////////////////////////////////
void DroneDemoWorldPlugin::OnUpdate()
{
  if (this->timer.GetElapsed() > this->timeLimit)
  {
    this->world->Reset();
/*    msgs::WorldControl msg;
    msg.mutable_reset()->set_all(true);
    // msg.set_pause(true);
    this->worldControlPub->Publish(msg);*/
    return;
  }

  if (this->world->GetRunning() && !this->timer.GetRunning())
  {
    this->timer.Start();
  }
  return;

  // std::cerr << " timer " << this->timer.GetElapsed() << std::endl;
}
