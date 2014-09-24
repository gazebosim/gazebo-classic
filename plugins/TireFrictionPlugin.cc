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

#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/TireFrictionPluginPrivate.hh"
#include "plugins/TireFrictionPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TireFrictionPlugin)

/////////////////////////////////////////////////
TireFrictionPlugin::TireFrictionPlugin()
  : dataPtr(new TireFrictionPluginPrivate)
{
}

/////////////////////////////////////////////////
TireFrictionPlugin::~TireFrictionPlugin()
{
  // Destroy the private data structure.
  // Make sure this happens last.
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void TireFrictionPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;
  GZ_ASSERT(_model, "TireFrictionPlugin _model pointer is NULL");

  this->dataPtr->world = this->dataPtr->model->GetWorld();
  GZ_ASSERT(this->dataPtr->world, "TireFrictionPlugin world pointer is NULL");

  this->dataPtr->sdf = _sdf;
  GZ_ASSERT(_sdf, "TireFrictionPlugin _sdf pointer is NULL");

  if (_sdf->HasElement("link_name"))
  {
    std::string linkName = _sdf->Get<std::string>("link_name");
    this->dataPtr->link = this->dataPtr->model->GetLink(linkName);
  }
  else
  {
    // link_name not supplied, get first link from model
    this->dataPtr->link = this->dataPtr->model->GetLink();
  }
  GZ_ASSERT(this->dataPtr->link, "TireFrictionPlugin link pointer is NULL");

  if (_sdf->HasElement("collision_name"))
  {
    std::string collisionName = _sdf->Get<std::string>("collision_name");
    this->dataPtr->link = this->dataPtr->link->GetCollision(0);
  }
  GZ_ASSERT(this->dataPtr->collision,
    "TireFrictionPlugin collision pointer is NULL");
}

/////////////////////////////////////////////////
void TireFrictionPlugin::Init()
{
  this->dataPtr->node.reset(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->world->GetName());

  std::string topic;
  {
    physics::PhysicsEnginePtr physics =
      this->dataPtr->world->GetPhysicsEngine();
    topic = physics->GetContactManager()->CreateFilter(
      this->dataPtr->collision->GetScopedName(),
      this->dataPtr->collision->GetScopedName());
  }

  // Subscribe to the contact topic
  this->dataPtr->contactSub = this->dataPtr->node->Subscribe(topic,
    &TireFrictionPlugin::OnContacts, this);

  this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&TireFrictionPlugin::OnUpdate, this));
}

/////////////////////////////////////////////////
void TireFrictionPlugin::OnContacts(ConstContactsPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->dataPtr->mutex);
  this->dataPtr->newestContactsMsg = *_msg;
  this->dataPtr->newMsg = true;
}

/////////////////////////////////////////////////
void TireFrictionPlugin::OnUpdate()
{
  // Only update when there is a new contact message.
  if (!this->dataPtr->newMsg)
  {
    // Use time step to track wait time between messages.
    double dt = this->physics->GetMaxStepSize();
    this->dataPtr->newMsgWait += common::Time(dt);

    const common::Time messageTime(1, 0);
    if (this->dataPtr->newMsgWait > messageTime)
    {
      gzlog << "Waited "
            << this->dataPtr->newMsgWait.Double()
            << " s without a contact message"
            << std::endl;
      this->dataPtr->newMsgWait.Set(0, 0);
    }
    return;
  }

  // Copy contacts message so that mutex lock is short.
  msgs::Contacts contacts;
  {
    boost::mutex::scoped_lock lock(this->mutex);
    contacts = this->dataPtr->newestContactsMsg;
    this->dataPtr->newMsg = false;
    this->dataPtr->newMsgWait.Set(0, 0);
  }

  // Compute slip at contact points.
  // Compute friction from slip.
  // Set friction coefficient.
}
