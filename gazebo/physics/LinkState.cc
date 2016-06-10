/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License")
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

#include "gazebo/common/Exception.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/LinkState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
LinkState::LinkState()
: State()
{
}

/////////////////////////////////////////////////
LinkState::LinkState(const LinkPtr _link, const common::Time &_realTime,
                  const common::Time &_simTime, const uint64_t _iterations)
  : State(_link->GetName(), _realTime, _simTime, _iterations)
{
  this->pose = _link->GetWorldPose();
  this->velocity = math::Pose(_link->GetWorldLinearVel(),
                   math::Quaternion(_link->GetWorldAngularVel()));
  this->acceleration = math::Pose(_link->GetWorldLinearAccel(),
                       math::Quaternion(_link->GetWorldAngularAccel()));
  this->wrench = math::Pose(_link->GetWorldForce(), math::Quaternion());
}

/////////////////////////////////////////////////
LinkState::LinkState(const LinkPtr _link)
  : State(_link->GetName(), _link->GetWorld()->GetRealTime(),
          _link->GetWorld()->GetSimTime(), _link->GetWorld()->GetIterations())
{
  this->pose = _link->GetWorldPose();
  this->velocity = math::Pose(_link->GetWorldLinearVel(),
                   math::Quaternion(_link->GetWorldAngularVel()));
  this->acceleration = math::Pose(_link->GetWorldLinearAccel(),
                       math::Quaternion(_link->GetWorldAngularAccel()));
  this->wrench = math::Pose(_link->GetWorldForce(), math::Quaternion());

  // Disabling CollisionStates to improve performance. This information is
  // probably not required.
  //
  // // Create all the collision states.
  // Collision_V collisions = _link->GetCollisions();
  // for (Collision_V::const_iterator iter = collisions.begin();
  //      iter != collisions.end(); ++iter)
  // {
  //   this->collisionStates.push_back(CollisionState(*iter));
  // }
}

/////////////////////////////////////////////////
LinkState::LinkState(const sdf::ElementPtr _sdf)
  : State()
{
  this->Load(_sdf);
}

/////////////////////////////////////////////////
LinkState::~LinkState()
{
}

/////////////////////////////////////////////////
void LinkState::Load(const LinkPtr _link, const common::Time &_realTime,
    const common::Time &_simTime, const uint64_t _iterations)
{
  this->name = _link->GetName();
  this->wallTime = common::Time::GetWallTime();
  this->realTime = _realTime;
  this->simTime = _simTime;
  this->iterations = _iterations;

  this->pose = _link->GetWorldPose();
  this->velocity.Set(_link->GetWorldLinearVel(),
                     _link->GetWorldAngularVel());
  this->acceleration.Set(_link->GetWorldLinearAccel(),
                         _link->GetWorldAngularAccel());
  this->wrench.Set(_link->GetWorldForce(), math::Quaternion());
}

/////////////////////////////////////////////////
void LinkState::Load(const sdf::ElementPtr _elem)
{
  // Set the name
  this->name = _elem->Get<std::string>("name");

  // Set the link name
  if (_elem->HasElement("pose"))
    this->pose = _elem->Get<math::Pose>("pose");
  else
    this->pose.Set(0, 0, 0, 0, 0, 0);

  // Set the link velocity
  if (_elem->HasElement("velocity"))
    this->velocity = _elem->Get<math::Pose>("velocity");
  else
    this->velocity.Set(0, 0, 0, 0, 0, 0);

  // Set the link acceleration
  if (_elem->HasElement("acceleration"))
    this->acceleration = _elem->Get<math::Pose>("acceleration");
  else
    this->acceleration.Set(0, 0, 0, 0, 0, 0);

  // Set the link wrench
  if (_elem->HasElement("wrench"))
    this->wrench = _elem->Get<math::Pose>("wrench");
  else
    this->wrench.Set(0, 0, 0, 0, 0, 0);
}

/////////////////////////////////////////////////
const math::Pose &LinkState::GetPose() const
{
  return this->pose;
}

/////////////////////////////////////////////////
const math::Pose &LinkState::GetVelocity() const
{
  return this->velocity;
}

/////////////////////////////////////////////////
const math::Pose &LinkState::GetAcceleration() const
{
  return this->acceleration;
}

/////////////////////////////////////////////////
const math::Pose &LinkState::GetWrench() const
{
  return this->wrench;
}

/////////////////////////////////////////////////
unsigned int LinkState::GetCollisionStateCount() const
{
  return this->collisionStates.size();
}

/////////////////////////////////////////////////
CollisionState LinkState::GetCollisionState(unsigned int _index) const
{
  if (_index < this->collisionStates.size())
    return this->collisionStates[_index];

  gzthrow("Index is out of range");
  return CollisionState();
}

/////////////////////////////////////////////////
CollisionState LinkState::GetCollisionState(
    const std::string &_collisionName) const
{
  for (std::vector<CollisionState>::const_iterator
       iter = this->collisionStates.begin();
       iter != this->collisionStates.end(); ++iter)
  {
    if ((*iter).GetName() == _collisionName)
      return *iter;
  }

  gzthrow("Invalid collision name[" + _collisionName + "]");
  return CollisionState();
}

/////////////////////////////////////////////////
const std::vector<CollisionState> &LinkState::GetCollisionStates() const
{
  return this->collisionStates;
}

/////////////////////////////////////////////////
bool LinkState::IsZero() const
{
  // Disabled for efficiency
  // bool result = true;
  // for (std::vector<CollisionState>::const_iterator iter =
  //      this->collisionStates.begin();
  //      iter != this->collisionStates.end() && result; ++iter)
  // {
  //   result = result && (*iter).IsZero();
  // }

  // return result && this->pose == math::Pose::Zero;

  return this->pose == math::Pose::Zero;
}

/////////////////////////////////////////////////
LinkState &LinkState::operator=(const LinkState &_state)
{
  State::operator=(_state);

  // Copy the pose
  this->pose = _state.pose;

  // Copy the velocity
  this->velocity = _state.velocity;

  // Copy the acceleration
  this->acceleration = _state.acceleration;

  // Copy the wrench
  this->wrench = _state.wrench;

  // Disabled for efficiency
  // Clear the collision states
  // this->collisionStates.clear();
  //// Copy the collision states
  // for (std::vector<CollisionState>::const_iterator iter =
  //     _state.collisionStates.begin();
  //     iter != _state.collisionStates.end(); ++iter)
  // {
  //  this->collisionStates.push_back(*iter);
  // }

  return *this;
}

/////////////////////////////////////////////////
LinkState LinkState::operator-(const LinkState &_state) const
{
  LinkState result;

  result.name = this->name;

  result.pose.pos = this->pose.pos - _state.pose.pos;
  result.pose.rot = _state.pose.rot.GetInverse() * this->pose.rot;

  result.velocity = this->velocity - _state.velocity;
  result.acceleration = this->acceleration - _state.acceleration;
  result.wrench = this->wrench - _state.wrench;

  // Disabled for efficiency
  // Insert the collision differences
  // for (std::vector<CollisionState>::const_iterator iter =
  //      _state.collisionStates.begin();
  //      iter != _state.collisionStates.end(); ++iter)
  // {
  //   CollisionState state =
  //   this->GetCollisionState((*iter).GetName()) - *iter;
  //   if (!state.IsZero())
  //     result.collisionStates.push_back(state);
  // }

  return result;
}

/////////////////////////////////////////////////
LinkState LinkState::operator+(const LinkState &_state) const
{
  LinkState result;

  result.name = this->name;

  result.pose.pos = this->pose.pos + _state.pose.pos;
  result.pose.rot = _state.pose.rot * this->pose.rot;

  result.velocity = this->velocity + _state.velocity;
  result.acceleration = this->acceleration + _state.acceleration;
  result.wrench = this->wrench + _state.wrench;

  // Disabled for efficiency
  // Insert the collision differences
  // for (std::vector<CollisionState>::const_iterator iter =
  //      _state.collisionStates.begin();
  //      iter != _state.collisionStates.end(); ++iter)
  // {
  //   CollisionState state =
  //   this->GetCollisionState((*iter).GetName()) + *iter;
  //   result.collisionStates.push_back(state);
  // }

  return result;
}

/////////////////////////////////////////////////
void LinkState::FillSDF(sdf::ElementPtr _sdf)
{
  _sdf->ClearElements();

  _sdf->GetAttribute("name")->Set(this->name);
  _sdf->GetElement("pose")->Set(this->pose);
  _sdf->GetElement("velocity")->Set(this->velocity);
  _sdf->GetElement("acceleration")->Set(this->acceleration);
  _sdf->GetElement("wrench")->Set(this->wrench);

  // Disabled for efficiency
  // for (std::vector<CollisionState>::iterator iter =
  //      this->collisionStates.begin();
  //      iter != this->collisionStates.end(); ++iter)
  // {
  //   sdf::ElementPtr elem = _sdf->AddElement("collision");
  //   (*iter).FillSDF(elem);
  // }
}

/////////////////////////////////////////////////
void LinkState::SetWallTime(const common::Time &_time)
{
  State::SetWallTime(_time);

  for (std::vector<CollisionState>::iterator
       iter = this->collisionStates.begin();
       iter != this->collisionStates.end(); ++iter)
  {
    (*iter).SetWallTime(_time);
  }
}

/////////////////////////////////////////////////
void LinkState::SetRealTime(const common::Time &_time)
{
  State::SetRealTime(_time);

  for (std::vector<CollisionState>::iterator
       iter = this->collisionStates.begin();
       iter != this->collisionStates.end(); ++iter)
  {
    (*iter).SetRealTime(_time);
  }
}

/////////////////////////////////////////////////
void LinkState::SetSimTime(const common::Time &_time)
{
  State::SetSimTime(_time);

  for (std::vector<CollisionState>::iterator
       iter = this->collisionStates.begin();
       iter != this->collisionStates.end(); ++iter)
  {
    (*iter).SetSimTime(_time);
  }
}

/////////////////////////////////////////////////
void LinkState::SetIterations(const uint64_t _iterations)
{
  State::SetIterations(_iterations);

  for (auto &collisionState : this->collisionStates)
    collisionState.SetIterations(_iterations);
}
