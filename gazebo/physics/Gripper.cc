/*
 * Copyright 2012 Open Source Robotics Foundation
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
}

/////////////////////////////////////////////////
Gripper::~Gripper()
{
  this->model.reset();
  this->physics.reset();
  this->world.reset();
  this->connections.clear();
}

/////////////////////////////////////////////////
void Gripper::Load(rml::ElementPtr _rml)
{
  this->fixedJoint = this->physics->CreateJoint("revolute", this->model);

  rml::ElementPtr grasp_check = _rml->GetElement("grasp_check");
  this->min_contact_count = grasp_check->Get<unsigned int>("min_contact_count");
  this->attachSteps = grasp_check->Get<int>("attach_steps");
  this->detachSteps = grasp_check->Get<int>("detach_steps");

  rml::ElementPtr palmLinkElem = _rml->GetElement("palm_link");
  this->palmLink = this->model->GetLink(palmLinkElem->Get<std::string>());
  if (!this->palmLink)
    gzerr << "palm link [" << palmLinkElem->Get<std::string>()
          << "] not found!\n";

  rml::ElementPtr gripperLinkElem = _rml->GetElement("gripper_link");

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
      collision->SetContactsEnabled(true);

      this->collisions[collision->GetScopedName()] = collision;
    }
    gripperLinkElem = gripperLinkElem->GetNextElement("gripper_link");
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
  if (this->contacts.size() >= this->min_contact_count)
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

  this->contacts.clear();
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

  for (unsigned int i = 0; i < this->contacts.size(); ++i)
  {
    std::string name1 = this->contacts[i].collision1->GetScopedName();
    std::string name2 = this->contacts[i].collision2->GetScopedName();

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
void Gripper::OnContact(const std::string &/*_collisionName*/,
                        const physics::Contact &_contact)
{
  CollisionPtr collision1 = boost::dynamic_pointer_cast<Collision>(
        this->world->GetEntity(_contact.collision1->GetScopedName()));

  CollisionPtr collision2 = boost::dynamic_pointer_cast<Collision>(
        this->world->GetEntity(_contact.collision2->GetScopedName()));

  if ((collision1 && collision1->IsStatic()) ||
      (collision2 && collision2->IsStatic()))
  {
    return;
  }

  this->contacts.push_back(_contact);
}

/////////////////////////////////////////////////
void Gripper::ResetDiffs()
{
  for (unsigned int i = 0; i < 10; ++i)
    this->diffs[i] = GZ_DBL_MAX;
}
