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

#include <algorithm>
#include <string>

#include "gazebo/common/Assert.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/physics/ode/ODESurfaceParams.hh"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/transport/transport.hh"
#include "plugins/TireFrictionPluginPrivate.hh"
#include "plugins/TireFrictionPlugin.hh"

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(TireFrictionPlugin)

/////////////////////////////////////////////////
TireFrictionPlugin::TireFrictionPlugin()
  : dataPtr(new TireFrictionPluginPrivate)
{
  this->dataPtr->newMsg = false;
  this->dataPtr->frictionStatic  = 1.1;
  this->dataPtr->frictionDynamic = 1.0;
  this->dataPtr->slipStatic  = 0.1;
  this->dataPtr->slipDynamic = 0.2;
  this->dataPtr->speedStatic = 1.0;
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

  if (_sdf->HasElement("friction_static"))
    this->dataPtr->frictionStatic = _sdf->Get<double>("friction_static");
  if (_sdf->HasElement("friction_dynamic"))
    this->dataPtr->frictionDynamic = _sdf->Get<double>("friction_dynamic");
  if (_sdf->HasElement("slip_static"))
  {
    double valueCheck = _sdf->Get<double>("slip_static");
    if (valueCheck <= 0)
    {
      gzerr << "slip_static parameter value ["
            << valueCheck
            << "] must be positive, using default value ["
            << this->dataPtr->slipStatic
            << std::endl;
    }
    else
    {
      this->dataPtr->slipStatic = valueCheck;
    }
  }
  if (_sdf->HasElement("slip_dynamic"))
  {
    double valueCheck = _sdf->Get<double>("slip_dynamic");
    if (valueCheck <= this->dataPtr->slipStatic)
    {
      this->dataPtr->slipDynamic = this->dataPtr->slipStatic + 0.1;
      gzerr << "slip_dynamic parameter value ["
            << valueCheck
            << "] must be greater than slip_static ["
            << this->dataPtr->slipStatic
            << "], using slip_static + 0.1 ["
            << this->dataPtr->slipDynamic
            << std::endl;
    }
    else
    {
      this->dataPtr->slipDynamic = valueCheck;
    }
  }
  if (_sdf->HasElement("speed_static"))
  {
    double valueCheck = _sdf->Get<double>("speed_static");
    if (valueCheck <= 0)
    {
      gzerr << "speed_static parameter value ["
            << valueCheck
            << "] must be positive, using default value ["
            << this->dataPtr->speedStatic
            << std::endl;
    }
    else
    {
      this->dataPtr->speedStatic = valueCheck;
    }
  }
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
  }
  this->dataPtr->newMsgWait.Set(0, 0);

  // Compute slip at contact points.
  // For each contact point:
  // Compute slip velocity
  // * compute relative velocity between bodies at contact point
  // * subtract velocity component parallel to normal vector
  // * take sum of velocities, weighted by normal force
  // Compute reference velocity
  // * max velocity magnitude
  double scaledFriction = 0.0;
  double contactsNormalForceSum = 0.0;
  for (int i = 0; i < contacts.contact_size(); ++i)
  {
    // Get pointers to collision objects
    const msgs::Contact *contact = &contacts.contact(i);
    const std::string collision1(contact->collision1());
    const std::string collision2(contact->collision2());
    physics::CollisionPtr collPtr1 =
      boost::dynamic_pointer_cast<physics::Collision>(
      this->dataPtr->world->GetEntity(collision1));
    physics::CollisionPtr collPtr2 =
      boost::dynamic_pointer_cast<physics::Collision>(
      this->dataPtr->world->GetEntity(collision2));
    physics::LinkPtr link1 = collPtr1->GetLink();
    physics::LinkPtr link2 = collPtr2->GetLink();

    // compute velocity at each contact point
    if (contact->position_size() == 0 ||
        contact->position_size() != contact->normal_size() ||
        contact->position_size() != contact->wrench_size())
    {
      gzerr << "No contacts or invalid contact message"
            << std::endl;
      continue;
    }

    double scaledSlipSpeed = 0.0;
    double scaledReferenceSpeed = 0.0;
    double contactNormalForceSum = 0.0;
    for (int j = 0; j < contact->position_size(); ++j)
    {
      // Contact position in world coordinates.
      math::Vector3 position = msgs::Convert(contact->position(j));

      // Velocity of each link at contact point in world coordinates.
      math::Vector3 velocity1;
      math::Vector3 velocity2;
      {
        math::Pose linkPose = link1->GetWorldPose();
        math::Vector3 offset = position - linkPose.pos;
        velocity1 = link1->GetWorldLinearVel(offset, math::Quaternion());
      }
      {
        math::Pose linkPose = link2->GetWorldPose();
        math::Vector3 offset = position - linkPose.pos;
        velocity2 = link2->GetWorldLinearVel(offset, math::Quaternion());
      }

      // Relative link velocity at contact point.
      math::Vector3 slipVelocity = velocity1 - velocity2;

      // Subtract normal velocity component
      math::Vector3 normal = msgs::Convert(contact->normal(j));
      slipVelocity -= normal * slipVelocity.Dot(normal);

      // Scale slip speed by normal force
      double slipSpeed = slipVelocity.GetLength();
      double normalForce;
      {
        math::Vector3 force1 = msgs::Convert(
          contact->wrench(j).body_1_wrench().force());
        math::Quaternion rot1 = link1->GetWorldPose().rot;
        normalForce = rot1.RotateVector(force1).Dot(normal);
      }
      scaledSlipSpeed += slipSpeed * std::abs(normalForce);
      contactNormalForceSum += std::abs(normalForce);

      // Compute reference speed
      // max of absolute speed at contact points and at link origin
      double referenceSpeed =
        std::max(velocity1.GetLength(), velocity2.GetLength());
      referenceSpeed = std::max(referenceSpeed,
        link1->GetWorldLinearVel().GetLength());
      referenceSpeed = std::max(referenceSpeed,
        link2->GetWorldLinearVel().GetLength());
      scaledReferenceSpeed += referenceSpeed * std::abs(normalForce);
    }

    // Compute aggregate slip and reference speed (m/s)
    double slipSpeed = scaledSlipSpeed / contactNormalForceSum;
    double referenceSpeed = scaledReferenceSpeed / contactNormalForceSum;

    // Compute friction as a function of slip and reference speeds.
    double friction = this->ComputeFriction(slipSpeed, referenceSpeed);
    scaledFriction += friction * contactNormalForceSum;

    gzdbg << "contact.time "
          << common::Time(msgs::Convert(contact->time())).Double()
          << ", "
          << collision1
          << ", "
          << collision2
          << ", "
          << slipSpeed
          << ", "
          << referenceSpeed
          << ", "
          << contactNormalForceSum
          << std::endl;

   contactsNormalForceSum += contactNormalForceSum;
  }
  double friction = scaledFriction / contactsNormalForceSum;

  // Set friction coefficient.
  if (this->dataPtr->physics->GetType() == "ode")
  {
    physics::ODESurfaceParamsPtr surface =
      boost::dynamic_pointer_cast<physics::ODESurfaceParams>(
        this->dataPtr->collision->GetSurface());
    if (surface)
    {
      // ideally we should change fdir1 I think?
      surface->frictionPyramid.SetMuPrimary(friction);
      surface->frictionPyramid.SetMuSecondary(friction);
    }
    else
    {
      gzerr << "Setting friction failed" << std::endl;
    }
  }
  else
  {
    gzerr << "Only ODE is supported right now" << std::endl;
  }
}

/////////////////////////////////////////////////
// This is an example function for computing friction based on slip
// and reference speed.
double TireFrictionPlugin::ComputeFriction(const double _slipSpeed,
                                           const double _referenceSpeed)
{
  // For very low speeds, there can be numerical problems.
  // Thus don't compute friction based on slip if
  // reference speed is less than 50% of static speed;
  // just use static friction coefficient.
  if (std::abs(_referenceSpeed) < 0.5 * std::abs(this->dataPtr->speedStatic))
  {
    return this->dataPtr->frictionStatic;
  }

  // Compute slip ratio:
  double slipRatio = std::abs(_slipSpeed) / std::abs(_referenceSpeed);

  // Compute friction as function of slip:
  const double muStatic = std::abs(this->dataPtr->frictionStatic);
  const double muDynamic = std::abs(this->dataPtr->frictionDynamic);

  // note muDynamic value corresponds to slipRation >= slipDynamic,
  // so we only need two if statements to check other values
  double frictionFromSlip = muDynamic;
  if (slipRatio < this->dataPtr->slipStatic)
  {
    frictionFromSlip = muStatic * slipRatio / this->dataPtr->slipStatic;
  }
  else if (slipRatio < this->dataPtr->slipDynamic)
  {
    frictionFromSlip = muDynamic + (muStatic - muDynamic)
      / (this->dataPtr->slipStatic - this->dataPtr->slipDynamic)
      * (slipRatio - this->dataPtr->slipDynamic);
  }

  // Now that friction is computed from slip, do some additional smoothing
  // at moderate speeds (between 50% and 100% speedStatic)
  double speedRatio = std::abs(_referenceSpeed)
    / std::abs(this->dataPtr->speedStatic);
  if (speedRatio >= 0.5 && speedRatio < 1.0)
  {
    return (frictionFromSlip - this->dataPtr->frictionStatic)
      / 0.5 * (speedRatio - 0.5);
  }

  // Otherwise speeds are high enough, so return friction from slip.
  return frictionFromSlip;
}
