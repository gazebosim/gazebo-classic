/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "physics/Body.hh"
#include "physics/Collision.hh"
#include "physics/World.hh"
#include "physics/BodyState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
BodyState::BodyState()
: State()
{
}

/////////////////////////////////////////////////
BodyState::BodyState(const BodyPtr _body)
  : State(_body->GetName(), _body->GetWorld()->GetRealTime(),
          _body->GetWorld()->GetSimTime())
{
  for (unsigned int i = 0; i < _body->GetChildCount(); ++i)
  {
    this->collisionStates.push_back(_body->GetCollision(i)->GetState());
  }
  this->pose = _body->GetRelativePose();
}

/////////////////////////////////////////////////
BodyState::~BodyState()
{
}

/////////////////////////////////////////////////
void BodyState::Load(sdf::ElementPtr _elem)
{
  this->name = _elem->GetValueString("name");

  if (_elem->HasElement("pose"))
    this->pose = _elem->GetElement("pose")->GetValuePose("");
}

/////////////////////////////////////////////////
math::Pose BodyState::GetPose() const
{
  return this->pose;
}

/////////////////////////////////////////////////
unsigned int BodyState::GetCollisionStateCount() const
{
  return this->collisionStates.size();
}

/////////////////////////////////////////////////
CollisionState BodyState::GetCollisionState(unsigned int _index) const
{
  if (_index < this->collisionStates.size())
    return this->collisionStates[_index];
  else
    gzerr << "Index is out of range\n";

  return CollisionState();
}

/////////////////////////////////////////////////
CollisionState BodyState::GetCollisionState(
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
void BodyState::FillStateSDF(sdf::ElementPtr _elem)
{
  _elem->GetAttribute("name")->Set(this->GetName());
  _elem->GetOrCreateElement("pose")->GetValue()->Set(this->pose);
  _elem->GetOrCreateElement("velocity")->GetValue()->Set(this->velocity);

  /*for (std::vector<math::Pose>::iterator iter = this->forces.begin();
       iter != this->forces.end(); ++iter)
  {
    sdf::ElementPtr forceElem = _elem->AddElement("force");
  }*/
}

/////////////////////////////////////////////////
void BodyState::UpdateBodySDF(sdf::ElementPtr _elem)
{
  _elem->GetOrCreateElement("origin")->GetAttribute("pose")->Set(this->pose);
}
