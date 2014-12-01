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

  this->dataPtr->physics = this->dataPtr->world->GetPhysicsEngine();
  GZ_ASSERT(this->dataPtr->physics,
            "TireFrictionPlugin physics pointer is NULL");

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
    this->dataPtr->collision = this->dataPtr->link->GetCollision(collisionName);
  }
  GZ_ASSERT(this->dataPtr->collision,
    "TireFrictionPlugin collision pointer is NULL");
}

/////////////////////////////////////////////////
void TireFrictionPlugin::Init()
{
  this->dataPtr->node.reset(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->world->GetName());

  std::string topic =
    this->dataPtr->physics->GetContactManager()->CreateFilter(
      this->dataPtr->collision->GetScopedName(),
      this->dataPtr->collision->GetScopedName());

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
    double dt = this->dataPtr->physics->GetMaxStepSize();
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
    boost::mutex::scoped_lock lock(this->dataPtr->mutex);
    contacts = this->dataPtr->newestContactsMsg;
    this->dataPtr->newMsg = false;
    this->dataPtr->newMsgWait.Set(0, 0);
  }

  // Compute slip at contact points.
  // The following needs to be done differently,
  // instead of computing average position of contact points:
  // * compute relative slip velocity between bodies at each contact point
  // * somehow lump these into a scalar value of slip
  //   (perhaps normalizing by normal force magnitude)

  //  First compute average position and normal of contact points.
  math::Vector3 positionAverage;
  math::Vector3 normalAverage;
  math::Vector3 forceAverage;
  math::Vector3 torqueAverage;

  {
    int positionCount = 0;
    int normalCount = 0;
    int forceCount = 0;

    for (int i = 0; i < contacts.contact_size(); ++i)
    {
      const msgs::Contact *contact = &contacts.contact(i);
      for (int j = 0; j < contact->position_size(); ++j)
      {
        positionAverage += msgs::Convert(contact->position(j));
        positionCount++;
      }
      for (int j = 0; j < contact->normal_size(); ++j)
      {
        normalAverage += msgs::Convert(contact->normal(j));
        normalCount++;
      }

      for (int j = 0; j < contact->wrench_size(); ++j)
      {
        forceAverage += msgs::Convert(
            contact->wrench(j).body_1_wrench().force());
        torqueAverage += msgs::Convert(
            contact->wrench(j).body_1_wrench().torque());
        forceCount++;
      }
    }

    if (positionCount > 0)
    {
      positionAverage = positionAverage / positionCount;
    }
    if (normalCount > 0)
    {
      normalAverage = normalAverage / normalCount;
    }
    if (forceCount > 0)
    {
      forceAverage = forceAverage / forceCount;
      torqueAverage = torqueAverage / forceCount;
    }
  }

  // Then compute velocity on body at that average contact point.
  math::Vector3 contactPointVelocity;
  {
    math::Pose linkPose = this->dataPtr->link->GetWorldPose();
    math::Vector3 offset = positionAverage - linkPose.pos;
    contactPointVelocity =
      this->dataPtr->link->GetWorldLinearVel(offset, math::Quaternion());
  }

  // Compute contact point speed in tangential directions.
  double speedTangential;
  {
    math::Vector3 velocityTangential = contactPointVelocity -
      contactPointVelocity.Dot(normalAverage) * normalAverage;
    speedTangential = velocityTangential.GetLength();
  }

  // Then normalize that tangential speed somehow.
  // Use speed at origin of link frame.
  double slip;
  {
    double speed = this->dataPtr->link->GetWorldLinearVel().GetLength();
    const double speedMin = 0.1;
    if (speed < speedMin)
    {
      speed = speedMin;
    }
    slip = speedTangential / speed;
  }

  std::cout << "Slip[" << slip << "] Force[" << forceAverage << "] Torque[" << torqueAverage << "]\n";
  // Compute friction from slip.
  // Set friction coefficient.
}
