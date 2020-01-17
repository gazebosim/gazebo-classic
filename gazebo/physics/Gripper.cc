/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <functional>
#include <map>
#include <mutex>
#include <vector>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/msgs/msgs.hh"

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

/// \internal
/// \brief Private data class for Gripper
class gazebo::physics::GripperPrivate
{
  /// \brief Callback used when the gripper contacts an object.
  /// \param[in] _msg Message that contains contact information.
  public: void OnContacts(ConstContactsPtr &_msg);

  /// \brief Update the gripper.
  public: void OnUpdate();

  /// \brief Attach an object to the gripper.
  public: void HandleAttach();

  /// \brief Detach an object from the gripper.
  public: void HandleDetach();

  /// \brief A reset function.
  public: void ResetDiffs();

  /// \brief Model that contains this gripper.
  public: physics::ModelPtr model;

  /// \brief Pointer to the world.
  public: physics::WorldPtr world;

  /// \brief A fixed joint to connect the gripper to a grasped object.
  public: physics::JointPtr fixedJoint;

  /// \brief The base link for the gripper.
  public: physics::LinkPtr palmLink;

  /// \brief All our connections.
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief The collisions for the links in the gripper.
  public: std::map<std::string, physics::CollisionPtr> collisions;

  /// \brief The current contacts.
  public: std::vector<msgs::Contact> contacts;

  /// \brief Mutex used to protect reading/writing the contact message.
  public: std::mutex mutexContacts;

  /// \brief True if the gripper has an object.
  public: bool attached;

  /// \brief Previous difference between the palm link and grasped
  /// object.
  public: ignition::math::Pose3d prevDiff;

  /// \brief Used to determine when to create the fixed joint.
  public: std::vector<double> diffs;

  /// \brief Current index into the diff array.
  public: int diffIndex;

  /// \brief Rate at which to update the gripper.
  public: common::Time updateRate;

  /// \brief Previous time when the gripper was updated.
  public: common::Time prevUpdateTime;

  /// \brief Number of iterations the gripper was contacting the same
  /// object.
  public: int posCount;

  /// \brief Number of iterations the gripper was not contacting the same
  /// object.
  public: int zeroCount;

  /// \brief Minimum number of links touching.
  public: unsigned int minContactCount;

  /// \brief Steps touching before engaging fixed joint
  public: int attachSteps;

  /// \brief Steps not touching before disengaging fixed joint
  public: int detachSteps;

  /// \brief Name of the gripper.
  public: std::string name;

  /// \brief Node for communication.
  public: transport::NodePtr node;

  /// \brief Subscription to contact messages from the physics engine.
  public: transport::SubscriberPtr contactSub;
};

/////////////////////////////////////////////////
Gripper::Gripper(ModelPtr _model) : dataPtr(new GripperPrivate)
{
  this->dataPtr->model = _model;
  this->dataPtr->world = this->dataPtr->model->GetWorld();

  this->dataPtr->diffs.resize(10);
  this->dataPtr->diffIndex = 0;
  this->dataPtr->ResetDiffs();

  this->dataPtr->attached = false;

  this->dataPtr->updateRate = common::Time(0, common::Time::SecToNano(0.75));

  this->dataPtr->node = transport::NodePtr(new transport::Node());
}

/////////////////////////////////////////////////
Gripper::~Gripper()
{
  if (this->dataPtr->world && this->dataPtr->world->Running())
  {
    physics::ContactManager *mgr =
        this->dataPtr->world->Physics()->GetContactManager();
    mgr->RemoveFilter(this->Name());
  }

  this->dataPtr->model.reset();
  this->dataPtr->world.reset();
  this->dataPtr->connections.clear();
}

/////////////////////////////////////////////////
void Gripper::Load(sdf::ElementPtr _sdf)
{
  this->dataPtr->node->Init(this->dataPtr->world->Name());

  this->dataPtr->name = _sdf->Get<std::string>("name");
  this->dataPtr->fixedJoint =
      this->dataPtr->world->Physics()->CreateJoint("fixed",
          this->dataPtr->model);

  this->dataPtr->fixedJoint->SetName(
      this->dataPtr->model->GetName() + "__gripper_fixed_joint__");

  sdf::ElementPtr graspCheck = _sdf->GetElement("grasp_check");
  this->dataPtr->minContactCount =
    graspCheck->Get<unsigned int>("min_contact_count");
  this->dataPtr->attachSteps = graspCheck->Get<int>("attach_steps");
  this->dataPtr->detachSteps = graspCheck->Get<int>("detach_steps");

  sdf::ElementPtr palmLinkElem = _sdf->GetElement("palm_link");
  this->dataPtr->palmLink =
    this->dataPtr->model->GetLink(palmLinkElem->Get<std::string>());

  if (!this->dataPtr->palmLink)
  {
    gzerr << "palm link [" << palmLinkElem->Get<std::string>()
          << "] not found!\n";
  }

  sdf::ElementPtr gripperLinkElem = _sdf->GetElement("gripper_link");

  while (gripperLinkElem)
  {
    physics::LinkPtr gripperLink
      = this->dataPtr->model->GetLink(gripperLinkElem->Get<std::string>());
    for (unsigned int j = 0; j < gripperLink->GetChildCount(); ++j)
    {
      physics::CollisionPtr collision = gripperLink->GetCollision(j);
      std::map<std::string, physics::CollisionPtr>::iterator collIter
        = this->dataPtr->collisions.find(collision->GetScopedName());
      if (collIter != this->dataPtr->collisions.end())
        continue;

      this->dataPtr->collisions[collision->GetScopedName()] = collision;
    }
    gripperLinkElem = gripperLinkElem->GetNextElement("gripper_link");
  }

  if (!this->dataPtr->collisions.empty())
  {
    // request the contact manager to publish messages to a custom topic for
    // this sensor
    physics::ContactManager *mgr =
        this->dataPtr->world->Physics()->GetContactManager();
    std::string topic = mgr->CreateFilter(this->Name(),
        this->dataPtr->collisions);
    if (!this->dataPtr->contactSub)
    {
      this->dataPtr->contactSub = this->dataPtr->node->Subscribe(topic,
          &GripperPrivate::OnContacts, this->dataPtr.get());
    }
  }
  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateEnd(
          std::bind(&GripperPrivate::OnUpdate, this->dataPtr.get())));
}

/////////////////////////////////////////////////
void Gripper::Init()
{
  this->dataPtr->prevUpdateTime = common::Time::GetWallTime();
  this->dataPtr->zeroCount = 0;
  this->dataPtr->posCount = 0;
  this->dataPtr->attached = false;
}

/////////////////////////////////////////////////
std::string Gripper::Name() const
{
  return this->dataPtr->name;
}

/////////////////////////////////////////////////
bool Gripper::IsAttached() const
{
  return this->dataPtr->attached;
}

/////////////////////////////////////////////////
void GripperPrivate::OnUpdate()
{
  if (common::Time::GetWallTime() - this->prevUpdateTime < this->updateRate)
  {
    return;
  }

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
  {
    this->HandleAttach();
  }
  else if (this->zeroCount > this->detachSteps && this->attached)
  {
    this->HandleDetach();
  }

  {
    std::lock_guard<std::mutex> lock(this->mutexContacts);
    this->contacts.clear();
  }

  this->prevUpdateTime = common::Time::GetWallTime();
}

/////////////////////////////////////////////////
void GripperPrivate::HandleAttach()
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
          this->world->EntityByName(name1));
      contactCounts[name1] += 1;
    }

    if (this->collisions.find(name2) == this->collisions.end())
    {
      cc[name2] = boost::dynamic_pointer_cast<Collision>(
          this->world->EntityByName(name2));
      contactCounts[name2] += 1;
    }
  }

  iter = contactCounts.begin();
  while (iter != contactCounts.end())
  {
    if (iter->second < 2)
    {
      contactCounts.erase(iter++);
    }
    else
    {
      if (!this->attached && cc[iter->first])
      {
        ignition::math::Pose3d diff =
          cc[iter->first]->GetLink()->WorldPose() - this->palmLink->WorldPose();

        double dd = (diff - this->prevDiff).Pos().SquaredLength();

        this->prevDiff = diff;

        this->diffs[this->diffIndex] = dd;
        double var = ignition::math::variance<double>(this->diffs);
        double max = ignition::math::max<double>(this->diffs);

        if (var < 1e-5 && max < 1e-5)
        {
          this->attached = true;

          this->fixedJoint->Load(this->palmLink,
              cc[iter->first]->GetLink(), ignition::math::Pose3d());
          this->fixedJoint->Init();
        }

        this->diffIndex = (this->diffIndex+1) % 10;
      }
      ++iter;
    }
  }
}

/////////////////////////////////////////////////
void GripperPrivate::HandleDetach()
{
  this->attached = false;
  this->fixedJoint->Detach();
}

/////////////////////////////////////////////////
void GripperPrivate::OnContacts(ConstContactsPtr &_msg)
{
  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    CollisionPtr collision1 = boost::dynamic_pointer_cast<Collision>(
        this->world->EntityByName(_msg->contact(i).collision1()));
    CollisionPtr collision2 = boost::dynamic_pointer_cast<Collision>(
        this->world->EntityByName(_msg->contact(i).collision2()));

    if ((collision1 && !collision1->IsStatic()) &&
        (collision2 && !collision2->IsStatic()))
    {
      std::lock_guard<std::mutex> lock(this->mutexContacts);
      this->contacts.push_back(_msg->contact(i));
    }
  }
}

/////////////////////////////////////////////////
void GripperPrivate::ResetDiffs()
{
  for (unsigned int i = 0; i < 10; ++i)
    this->diffs[i] = ignition::math::MAX_D;
}


