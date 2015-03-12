/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/JointState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
JointState::JointState()
: State()
{
}

/////////////////////////////////////////////////
JointState::JointState(JointPtr _joint, const common::Time &_realTime,
    const common::Time &_simTime)
: State(_joint->GetName(), _realTime, _simTime)
{
  // Set the joint angles.
  for (unsigned int i = 0; i < _joint->GetAngleCount(); ++i)
    this->angles.push_back(_joint->GetAngle(i));
}

/////////////////////////////////////////////////
JointState::JointState(JointPtr _joint)
: State(_joint->GetName(), _joint->GetWorld()->GetRealTime(),
        _joint->GetWorld()->GetSimTime())
{
  // Set the joint angles.
  for (unsigned int i = 0; i < _joint->GetAngleCount(); ++i)
    this->angles.push_back(_joint->GetAngle(i));
}

/////////////////////////////////////////////////
JointState::JointState(const sdf::ElementPtr _sdf)
  : State()
{
  // Load the state from SDF
  this->Load(_sdf);
}

/////////////////////////////////////////////////
JointState::~JointState()
{
}

/////////////////////////////////////////////////
void JointState::Load(JointPtr _joint, const common::Time &_realTime,
    const common::Time &_simTime)
{
  this->name = _joint->GetName();
  this->realTime = _realTime;
  this->simTime = _simTime;
  this->wallTime = common::Time::GetWallTime();

  // Set the joint angles.
  for (unsigned int i = 0; i < _joint->GetAngleCount(); ++i)
    this->angles.push_back(_joint->GetAngle(i));
}

/////////////////////////////////////////////////
void JointState::Load(const sdf::ElementPtr _elem)
{
  // Set the name
  this->name = _elem->Get<std::string>("name");

  // Set the angles
  this->angles.clear();
  if (_elem->HasElement("angle"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("angle");
    while (childElem)
    {
      unsigned int axis = childElem->Get<unsigned int>("axis");
      if (axis+1 > this->angles.size())
        this->angles.resize(axis+1, math::Angle(0.0));
      this->angles[axis] = childElem->Get<double>();
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
  if (_axis < this->angles.size())
    return this->angles[_axis];

  gzthrow("Index[" + boost::lexical_cast<std::string>(_axis) +
          "] is out of range.");
  return math::Angle();
}

/////////////////////////////////////////////////
const std::vector<math::Angle> &JointState::GetAngles() const
{
  return this->angles;
}

/////////////////////////////////////////////////
bool JointState::IsZero() const
{
  bool result = true;
  for (std::vector<math::Angle>::const_iterator iter = this->angles.begin();
       iter != this->angles.end() && result; ++iter)
  {
    result = result && (*iter) == math::Angle::Zero;
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
  for (std::vector<math::Angle>::const_iterator iter = _state.angles.begin();
       iter != _state.angles.end(); ++iter)
  {
    this->angles.push_back(*iter);
  }

  return *this;
}

/////////////////////////////////////////////////
JointState JointState::operator-(const JointState &_state) const
{
  JointState result;

  // Set the name
  result.name = this->name;

  // Set the angles.
  /// \TODO: this will produce incorrect results if _state doesn't have the
  /// same set of angles as *this.
  int i = 0;
  for (std::vector<math::Angle>::const_iterator iterA = this->angles.begin(),
       iterB = _state.angles.begin(); iterA != this->angles.end() &&
       iterB != _state.angles.end(); ++iterA, ++iterB, ++i)
  {
    result.angles.push_back((*iterA) - (*iterB));
  }

  return result;
}

/////////////////////////////////////////////////
JointState JointState::operator+(const JointState &_state) const
{
  JointState result;

  // Set the name
  result.name = this->name;

  // Set the angles.
  /// \TODO: this will produce incorrect results if _state doesn't have the
  /// same set of angles as *this.
  int i = 0;
  for (std::vector<math::Angle>::const_iterator iterA = this->angles.begin(),
       iterB = _state.angles.begin(); iterA != this->angles.end() &&
       iterB != _state.angles.end(); ++iterA, ++iterB, ++i)
  {
    result.angles.push_back((*iterA) + (*iterB));
  }

  return result;
}

/////////////////////////////////////////////////
void JointState::FillSDF(sdf::ElementPtr _sdf)
{
  _sdf->ClearElements();

  _sdf->GetAttribute("name")->Set(this->name);

  int i = 0;
  for (std::vector<math::Angle>::const_iterator iter = this->angles.begin();
       iter != this->angles.end(); ++iter, ++i)
  {
    sdf::ElementPtr elem = _sdf->AddElement("angle");
    elem->GetAttribute("axis")->Set(i);
    elem->Set((*iter).Radian());
  }
}
