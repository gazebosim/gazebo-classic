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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/bind.hpp>

#include "gazebo/common/Events.hh"

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Subscriber.hh"

#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Contact.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Gripper.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
Gripper::Gripper(ModelPtr _model)
{
  this->model = _model;
  this->world = this->model->GetWorld();
  this->physics = this->world->GetPhysicsEngine();

  this->diffs.resize(10);
  this->diffIndex = 0;
  this->ResetDiffs();

  this->attached = false;

  this->updateRate = common::Time(0, common::Time::SecToNano(0.75));

  this->node = transport::NodePtr(new transport::Node());
}

/////////////////////////////////////////////////
Gripper::~Gripper()
{
  if (this->world && this->world->GetRunning())
  {
    physics::ContactManager *mgr =
        this->world->GetPhysicsEngine()->GetContactManager();
    mgr->RemoveFilter(this->GetName());
  }

  this->model.reset();
  this->physics.reset();
  this->world.reset();
  this->connections.clear();
}

/////////////////////////////////////////////////
void Gripper::Load(sdf::ElementPtr _sdf)
{
  this->node->Init(this->world->GetName());

  this->name = _sdf->Get<std::string>("name");
  this->fixedJoint = this->physics->CreateJoint("revolute", this->model);

  sdf::ElementPtr graspCheck = _sdf->GetElement("grasp_check");
  this->minContactCount = graspCheck->Get<unsigned int>("min_contact_count");
  this->attachSteps = graspCheck->Get<int>("attach_steps");
  this->detachSteps = graspCheck->Get<int>("detach_steps");

  sdf::ElementPtr palmLinkElem = _sdf->GetElement("palm_link");
  this->palmLink = this->model->GetLink(palmLinkElem->Get<std::string>());
  if (!this->palmLink)
    gzerr << "palm link [" << palmLinkElem->Get<std::string>()
          << "] not found!\n";

  sdf::ElementPtr gripperLinkElem = _sdf->GetElement("gripper_link");

  while (gripperLinkElem)
  {
    physics::LinkPtr gripperLink
      = this->model->GetLink(gripperLinkElem->Get<std::string>());
    for (unsigned int j = 0; j < gripperLink->GetChildCount(); ++j)
    {
      physics::CollisionPtr collision = gripperLink->GetCollision(j);
      std::map<std::string, physics::CollisionPtr>::iterator collIter
        = this->collisions.find(collision->GetScopedName());
      if (collIter != this->collisions.end())
        continue;

      this->collisions[collision->GetScopedName()] = collision;
    }
    gripperLinkElem = gripperLinkElem->GetNextElement("gripper_link");
  }

  if (!this->collisions.empty())
  {
    // request the contact manager to publish messages to a custom topic for
    // this sensor
    physics::ContactManager *mgr =
        this->world->GetPhysicsEngine()->GetContactManager();
    std::string topic = mgr->CreateFilter(this->GetName(), this->collisions);
    if (!this->contactSub)
    {
      this->contactSub = this->node->Subscribe(topic,
          &Gripper::OnContacts, this);
    }
  }
  this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
          boost::bind(&Gripper::OnUpdate, this)));
}

/////////////////////////////////////////////////
void Gripper::Init()
{
  this->prevUpdateTime = common::Time::GetWallTime();
  this->zeroCount = 0;
  this->posCount = 0;
  this->attached = false;
}

/////////////////////////////////////////////////
void Gripper::OnUpdate()
{
  if (common::Time::GetWallTime() - this->prevUpdateTime < this->updateRate)
    return;

  // @todo: should package the decision into a function
  if (this->contacts.size() >= this->minContactCount)
  {
    this->posCount++;
    this->zeroCount = 0;
  }
  else
  {
    this->zeroCount++;
    this->posCount = std::max(0, this->posCount-1);
  }

  if (this->posCount > this->attachSteps && !this->attached)
    this->HandleAttach();
  else if (this->zeroCount > this->detachSteps && this->attached)
    this->HandleDetach();

  {
    boost::mutex::scoped_lock lock(this->mutexContacts);
    this->contacts.clear();
  }

  this->prevUpdateTime = common::Time::GetWallTime();
}

/////////////////////////////////////////////////
void Gripper::HandleAttach()
{
  if (!this->palmLink)
  {
    gzwarn << "palm link not found, not enforcing grasp hack!\n";
    return;
  }

  std::map<std::string, physics::CollisionPtr> cc;
  std::map<std::string, int> contactCounts;
  std::map<std::string, int>::iterator iter;

  // This function is only called from the OnUpdate function so
  // the call to contacts.clear() is not going to happen in
  // parallel with the reads in the following code, no mutex
  // needed.
  for (unsigned int i = 0; i < this->contacts.size(); ++i)
  {
    std::string name1 = this->contacts[i].collision1();
    std::string name2 = this->contacts[i].collision2();

    if (this->collisions.find(name1) == this->collisions.end())
    {
      cc[name1] = boost::dynamic_pointer_cast<Collision>(
          this->world->GetEntity(name1));
      contactCounts[name1] += 1;
    }

    if (this->collisions.find(name2) == this->collisions.end())
    {
      cc[name2] = boost::dynamic_pointer_cast<Collision>(
          this->world->GetEntity(name2));
      contactCounts[name2] += 1;
    }
  }

  iter = contactCounts.begin();
  while (iter != contactCounts.end())
  {
    if (iter->second < 2)
      contactCounts.erase(iter++);
    else
    {
      if (!this->attached && cc[iter->first])
      {
        math::Pose diff = cc[iter->first]->GetLink()->GetWorldPose() -
          this->palmLink->GetWorldPose();

        double dd = (diff - this->prevDiff).pos.GetSquaredLength();

        this->prevDiff = diff;

        this->diffs[this->diffIndex] = dd;
        double var = math::variance<double>(this->diffs);
        double max = math::max<double>(this->diffs);

        if (var < 1e-5 && max < 1e-5)
        {
          this->attached = true;

          this->fixedJoint->Load(this->palmLink,
              cc[iter->first]->GetLink(), math::Pose());
          this->fixedJoint->Init();
          this->fixedJoint->SetHighStop(0, 0);
          this->fixedJoint->SetLowStop(0, 0);
        }

        this->diffIndex = (this->diffIndex+1) % 10;
      }
      ++iter;
    }
  }
}

/////////////////////////////////////////////////
void Gripper::HandleDetach()
{
  this->attached = false;
  this->fixedJoint->Detach();
}

/////////////////////////////////////////////////
void Gripper::OnContacts(ConstContactsPtr &_msg)
{
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    CollisionPtr collision1 = boost::dynamic_pointer_cast<Collision>(
        this->world->GetEntity(_msg->contact(i).collision1()));
    CollisionPtr collision2 = boost::dynamic_pointer_cast<Collision>(
        this->world->GetEntity(_msg->contact(i).collision2()));

    if ((collision1 && !collision1->IsStatic()) &&
        (collision2 && !collision2->IsStatic()))
    {
      boost::mutex::scoped_lock lock(this->mutexContacts);
      this->contacts.push_back(_msg->contact(i));
    }
  }
}

/////////////////////////////////////////////////
void Gripper::ResetDiffs()
{
  for (unsigned int i = 0; i < 10; ++i)
    this->diffs[i] = GZ_DBL_MAX;
}

/////////////////////////////////////////////////
std::string Gripper::GetName() const
{
  return this->name;
}

/////////////////////////////////////////////////
bool Gripper::IsAttached() const
{
  return this->attached;
}
