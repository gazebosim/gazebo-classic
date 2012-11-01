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
  for (unsigned int i = 0; i < _link->GetChildCount(); ++i)
  {
    this->collisionStates.push_back(_link->GetCollision(i)->GetState());
  }
  this->pose = _link->GetRelativePose();
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
    this->pose = _elem->GetElement("pose")->GetValuePose("");
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
void LinkState::FillStateSDF(sdf::ElementPtr _elem)
{
  _elem->GetAttribute("name")->Set(this->GetName());
  _elem->GetElement("pose")->GetValue()->Set(this->pose);
  _elem->GetElement("velocity")->GetValue()->Set(this->velocity);

  /*for (std::vector<math::Pose>::iterator iter = this->forces.begin();
       iter != this->forces.end(); ++iter)
  {
    sdf::ElementPtr forceElem = _elem->AddElement("force");
  }*/
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

  // Clear the forces
  this->forces.clear();

  // Clear the collision states
  this->collisionStates.clear();

  // Copy the forces
  for (std::vector<math::Pose>::const_iterator iter = _state.forces.begin();
       iter != _state.forces.end(); ++iter)
  {
    this->forces.push_back(*iter);
  }

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
LinkState LinkState::operator-(const LinkState &_state) const
{
  LinkState result;

  result.pose = this->pose - _state.pose;
  result.velocity = this->velocity - _state.velocity;
  result.acceleration = this->acceleration - _state.acceleration;

  // Insert the force differences
  for (std::vector<math::Pose>::const_iterator
       iterA = this->forces.begin(), iterB = _state.forces.begin();
       iterA != this->forces.end() && iterB != _state.forces.end();
       ++iterA, ++iterB)
  {
    result.forces.push_back((*iterA) - (*iterB));
  }

  // Insert the collision differences
  for (std::vector<CollisionState>::const_iterator iter =
       _state.collisionStates.begin();
       iter != _state.collisionStates.end(); ++iter)
  {
    result.collisionStates.push_back(
        this->GetCollisionState((*iter).GetName()) - *iter);
  }

  return result;
}
