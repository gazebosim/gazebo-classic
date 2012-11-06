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

#include "physics/Joint.hh"
#include "physics/Link.hh"
#include "physics/World.hh"
#include "physics/JointState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
JointState::JointState()
: State()
{
}

/////////////////////////////////////////////////
JointState::JointState(JointPtr _joint)
: State(_joint->GetName(), _joint->GetWorld()->GetRealTime(),
        _joint->GetWorld()->GetSimTime())
{
  this->angles[0] = _joint->GetAngle(0);
  this->angles[1] = _joint->GetAngle(1);
}

/////////////////////////////////////////////////
JointState::~JointState()
{
}

/////////////////////////////////////////////////
void JointState::Load(sdf::ElementPtr _elem)
{
  this->name = _elem->GetValueString("name");

  this->angles.clear();
  if (_elem->HasElement("angle"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("angle");
    while (childElem)
    {
      int axis = childElem->GetValueInt("axis");
      double angle = childElem->GetValueDouble();
      this->angles[axis] = angle;
      childElem = childElem->GetNextElement("angle");
    }
  }

}

/////////////////////////////////////////////////
unsigned int JointState::GetAngleCount() const
{
  return this->angles.size();
}

/////////////////////////////////////////////////
math::Angle JointState::GetAngle(unsigned int _axis) const
{
  math::Angle angle;

  std::map<int, math::Angle>::const_iterator iter = this->angles.find(_axis);
  if (iter != this->angles.end())
    angle = iter->second;
  else
    gzerr << "Index[" << _axis << "] is out of range.\n";

  return angle;
}

/////////////////////////////////////////////////
bool JointState::IsZero() const
{
  bool result = true;
  for (std::map<int, math::Angle>::const_iterator iter = this->angles.begin();
       iter != this->angles.end() && result; ++iter)
  {
    result = result && iter->second == math::Angle(0.0);
  }

  return result;
}

/////////////////////////////////////////////////
JointState &JointState::operator=(const JointState &_state)
{
  State::operator=(_state);

  // Clear the angles.
  this->angles.clear();

  // Copy the angles.
  for (std::map<int, math::Angle>::const_iterator iter = _state.angles.begin();
       iter != _state.angles.end(); ++iter)
  {
    this->angles[iter->first] = iter->second;
  }

  return *this;
}

/////////////////////////////////////////////////
JointState JointState::operator-(const JointState &_state) const
{
  JointState result = *this;

  result.angles.clear();

  // Copy the angles.
  /// \TODO: this will produce incorrect results if _state doesn't have the
  /// same set of angles as *this.
  int i = 0;
  for (std::map<int, math::Angle>::const_iterator iterA = this->angles.begin(),
       iterB = _state.angles.begin(); iterA != this->angles.end() &&
       iterB != _state.angles.end(); ++iterA, ++iterB, ++i)
  {
    result.angles[i] = iterA->second - iterB->second;
  }

  return result;
}

/////////////////////////////////////////////////
JointState JointState::operator+(const JointState &_state) const
{
  JointState result = *this;

  result.angles.clear();

  // Copy the angles.
  /// \TODO: this will produce incorrect results if _state doesn't have the
  /// same set of angles as *this.
  int i = 0;
  for (std::map<int, math::Angle>::const_iterator iterA = this->angles.begin(),
       iterB = _state.angles.begin(); iterA != this->angles.end() &&
       iterB != _state.angles.end(); ++iterA, ++iterB, ++i)
  {
    result.angles[i] = iterA->second + iterB->second;
  }

  return result;
}
