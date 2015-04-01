/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include "gazebo/transport/transport.hh"
#include "plugins/HydraDemoPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(HydraDemoPlugin)

/////////////////////////////////////////////////
HydraDemoPlugin::HydraDemoPlugin()
{
}

/////////////////////////////////////////////////
HydraDemoPlugin::~HydraDemoPlugin()
{
}

/////////////////////////////////////////////////
void HydraDemoPlugin::OnHydra0(ConstHydraPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->msgMutex);
  this->hydraMsgPtr0 = _msg;
}

/////////////////////////////////////////////////
void HydraDemoPlugin::OnHydra1(ConstHydraPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->msgMutex);
  this->hydraMsgPtr1 = _msg;
}

/////////////////////////////////////////////////
void HydraDemoPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Get the world name.
  this->model = _parent;
  this->world = this->model->GetWorld();

  // Subscribe to Hydra updates by registering OnHydra() callback.
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());
  this->hydraSub0 = this->node->Subscribe("~/hydra0",
      &HydraDemoPlugin::OnHydra0, this);

  this->hydraSub1 = this->node->Subscribe("~/hydra1",
      &HydraDemoPlugin::OnHydra0, this);

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&HydraDemoPlugin::Update, this, _1));
}

/////////////////////////////////////////////////
void HydraDemoPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  boost::mutex::scoped_lock lock(this->msgMutex);

  // Return if we don't have messages yet
  if (!this->hydraMsgPtr0)
    return;

  // Read the value of the right joystick.
  double joyX = this->hydraMsgPtr0->right().joy_x();
  double joyY = this->hydraMsgPtr0->right().joy_y();

  // Move the model.
  this->model->SetLinearVel(math::Vector3(-joyX * 0.2, joyY * 0.2, 0));

  // Remove the message that has been processed.
  this->hydraMsgPtr0.reset();

  // Return if we don't have messages yet
  if (!this->hydraMsgPtr1)
    return;

  // Read the value of the right joystick.
  joyX = this->hydraMsgPtr1->right().joy_x();
  joyY = this->hydraMsgPtr1->right().joy_y();

  // Move the model.
  this->model->SetLinearVel(math::Vector3(-joyX * 0.2, joyY * 0.2, 0));

  // Remove the message that has been processed.
  this->hydraMsgPtr1.reset();
}
