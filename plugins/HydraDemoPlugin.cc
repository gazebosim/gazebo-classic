/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
void HydraDemoPlugin::OnHydra(ConstHydraPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->msgMutex);
  this->hydraMsgPtr = _msg;
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
  this->hydraSub = this->node->Subscribe("~/hydra",
      &HydraDemoPlugin::OnHydra, this);

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
  if (!this->hydraMsgPtr)
    return;

  // Read the value of the right joystick.
  double joyX = this->hydraMsgPtr->right().joy_x();
  double joyY = this->hydraMsgPtr->right().joy_y();

  // Move the model.
  this->model->SetLinearVel(math::Vector3(-joyX * 0.2, joyY * 0.2, 0));

  // Remove the message that has been processed.
  this->hydraMsgPtr.reset();
}
