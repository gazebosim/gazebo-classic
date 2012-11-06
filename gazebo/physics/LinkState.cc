/*
 * Copyright 2011 Nate Koenig
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

#include "physics/Link.hh"
#include "physics/Collision.hh"
#include "physics/World.hh"
#include "physics/LinkState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
LinkState::LinkState()
: State()
{
}

/////////////////////////////////////////////////
LinkState::LinkState(const LinkPtr _link)
  : State(_link->GetName(), _link->GetWorld()->GetRealTime(),
          _link->GetWorld()->GetSimTime())
{
  this->pose = _link->GetRelativePose();
  this->velocity = math::Pose(_link->GetRelativeLinearVel(),
      math::Quaternion(_link->GetRelativeAngularVel()));
  this->acceleration = math::Pose(_link->GetRelativeLinearAccel(),
      math::Quaternion(_link->GetRelativeAngularAccel()));
  this->forceMag = math::Pose(_link->GetRelativeForce(), math::Quaternion());

  for (unsigned int i = 0; i < _link->GetChildCount(); ++i)
  {
    CollisionPtr coll = _link->GetCollision(i);
    if (coll)
      this->collisionStates.push_back(CollisionState(coll));
  }
}

/////////////////////////////////////////////////
LinkState::~LinkState()
{
}

/////////////////////////////////////////////////
void LinkState::Load(sdf::ElementPtr _elem)
{
  this->name = _elem->GetValueString("name");

  if (_elem->HasElement("pose"))
    this->pose = _elem->GetValuePose("pose");

  if (_elem->HasElement("velocity"))
    this->velocity = _elem->GetValuePose("velocity");

  if (_elem->HasElement("acceleration"))
    this->acceleration = _elem->GetValuePose("acceleration");

  if (_elem->HasElement("wrench"))
  {
    this->forcePos = _elem->GetElement("wrench")->GetValueVector3("pos");
    this->forceMag = _elem->GetElement("wrench")->GetValuePose("mag");
  }
}

/////////////////////////////////////////////////
math::Pose LinkState::GetPose() const
{
  return this->pose;
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
  else
    gzerr << "Index is out of range\n";

  return CollisionState();
}

/////////////////////////////////////////////////
CollisionState LinkState::GetCollisionState(
    const std::string &_collisionName) const
{
  std::vector<CollisionState>::const_iterator iter;
  for (iter = this->collisionStates.begin();
       iter != this->collisionStates.end(); ++iter)
  {
    if ((*iter).GetName() == _collisionName)
      return *iter;
  }

  return CollisionState();
}

/////////////////////////////////////////////////
void LinkState::FillStateSDF(sdf::ElementPtr _elem) const
{
  _elem->GetAttribute("name")->Set(this->GetName());
  _elem->GetElement("pose")->Set(this->pose);
  _elem->GetElement("velocity")->Set(this->velocity);
  _elem->GetElement("wrench")->GetElement("pos")->Set(this->forcePos);
  _elem->GetElement("wrench")->GetElement("mag")->Set(this->forceMag);

  for (std::vector<CollisionState>::const_iterator iter =
      this->collisionStates.begin();
      iter != this->collisionStates.end(); ++iter)
  {
    sdf::ElementPtr elem = _elem->AddElement("collision");
    (*iter).FillStateSDF(elem);
  }
}

/////////////////////////////////////////////////
void LinkState::UpdateLinkSDF(sdf::ElementPtr _elem)
{
  _elem->GetElement("pose")->Set(this->pose);
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

  // Copy the force
  this->forcePos = _state.forcePos;
  this->forceMag = _state.forceMag;

  // Clear the collision states
  this->collisionStates.clear();

  // Copy the collision states
  for (std::vector<CollisionState>::const_iterator iter =
       _state.collisionStates.begin();
       iter != _state.collisionStates.end(); ++iter)
  {
    this->collisionStates.push_back(*iter);
  }

  return *this;
}

/////////////////////////////////////////////////
bool LinkState::IsZero() const
{
  bool result = true;

  for (std::vector<CollisionState>::const_iterator iter =
       this->collisionStates.begin();
       iter != this->collisionStates.end() && result; ++iter)
  {
    result = result && (*iter).IsZero();
  }

  return result && this->pose == math::Pose::Zero &&
         this->velocity == math::Pose::Zero &&
         this->acceleration == math::Pose::Zero &&
         this->forceMag == math::Pose::Zero;
}

/////////////////////////////////////////////////
LinkState LinkState::operator-(const LinkState &_state) const
{
  LinkState result = *this;

  result.pose -= _state.pose;
  result.velocity -= _state.velocity;
  result.acceleration -= _state.acceleration;
  result.forceMag -= _state.forceMag;
  result.forcePos -= _state.forcePos;

  result.collisionStates.clear();

  // Insert the collision differences
  for (std::vector<CollisionState>::const_iterator iter =
       _state.collisionStates.begin();
       iter != _state.collisionStates.end(); ++iter)
  {
    CollisionState state = this->GetCollisionState((*iter).GetName()) - *iter;
    if (!state.IsZero())
      result.collisionStates.push_back(state);
  }

  return result;
}

/////////////////////////////////////////////////
LinkState LinkState::operator+(const LinkState &_state) const
{
  LinkState result = *this;

  result.pose += _state.pose;
  result.velocity += _state.velocity;
  result.acceleration += _state.acceleration;
  result.forceMag += _state.forceMag;
  result.forcePos += _state.forcePos;

  result.collisionStates.clear();

  // Insert the collision differences
  for (std::vector<CollisionState>::const_iterator iter =
       _state.collisionStates.begin();
       iter != _state.collisionStates.end(); ++iter)
  {
    CollisionState state = this->GetCollisionState((*iter).GetName()) + *iter;
    result.collisionStates.push_back(state);
  }

  return result;
}
