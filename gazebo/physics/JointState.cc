/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <ignition/math/Angle.hh>
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
    const common::Time &_simTime, const uint64_t _iterations)
: State(_joint->GetName(), _realTime, _simTime, _iterations)
{
  // Set the joint positions.
  for (unsigned int i = 0; i < _joint->DOF(); ++i)
    this->positions.push_back(_joint->Position(i));
}

/////////////////////////////////////////////////
JointState::JointState(JointPtr _joint)
: State(_joint->GetName(), _joint->GetWorld()->RealTime(),
        _joint->GetWorld()->SimTime(), _joint->GetWorld()->Iterations())
{
  // Set the joint positions.
  for (unsigned int i = 0; i < _joint->DOF(); ++i)
    this->positions.push_back(_joint->Position(i));
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

  // Set the joint positions.
  for (unsigned int i = 0; i < _joint->DOF(); ++i)
    this->positions.push_back(_joint->Position(i));
}

/////////////////////////////////////////////////
void JointState::Load(const sdf::ElementPtr _elem)
{
  // Set the name
  this->name = _elem->Get<std::string>("name");

  // Set the positions
  this->positions.clear();
  if (_elem->HasElement("angle"))
  {
    sdf::ElementPtr childElem = _elem->GetElement("angle");
    while (childElem)
    {
      unsigned int axis = childElem->Get<unsigned int>("axis");
      if (axis+1 > this->positions.size())
        this->positions.resize(axis+1, 0.0);
      this->positions[axis] = childElem->Get<double>();
      childElem = childElem->GetNextElement("angle");
    }
  }
}

/////////////////////////////////////////////////
unsigned int JointState::GetAngleCount() const
{
  return this->positions.size();
}

/////////////////////////////////////////////////
double JointState::Position(const unsigned int _axis) const
{
  if (_axis < this->positions.size())
    return this->positions[_axis];

  return ignition::math::NAN_D;
}

/////////////////////////////////////////////////
const std::vector<double> &JointState::Positions() const
{
  return this->positions;
}

/////////////////////////////////////////////////
bool JointState::IsZero() const
{
  bool result = true;
  for (std::vector<double>::const_iterator iter =
       this->positions.begin(); iter != this->positions.end() && result; ++iter)
  {
    result = result && ignition::math::equal((*iter), 0.0);
  }

  return result;
}

/////////////////////////////////////////////////
JointState &JointState::operator=(const JointState &_state)
{
  State::operator=(_state);

  // Clear the positions.
  this->positions.clear();

  // Copy the positions.
  for (std::vector<double>::const_iterator iter =
       _state.positions.begin(); iter != _state.positions.end(); ++iter)
  {
    this->positions.push_back(*iter);
  }

  return *this;
}

/////////////////////////////////////////////////
JointState JointState::operator-(const JointState &_state) const
{
  JointState result;

  // Set the name
  result.name = this->name;

  // Set the positions.
  /// \TODO: this will produce incorrect results if _state doesn't have the
  /// same set of positions as *this.
  int i = 0;
  for (std::vector<double>::const_iterator iterA =
       this->positions.begin(), iterB = _state.positions.begin();
       iterA != this->positions.end() &&
       iterB != _state.positions.end(); ++iterA, ++iterB, ++i)
  {
    result.positions.push_back((*iterA) - (*iterB));
  }

  return result;
}

/////////////////////////////////////////////////
JointState JointState::operator+(const JointState &_state) const
{
  JointState result;

  // Set the name
  result.name = this->name;

  // Set the positions.
  /// \TODO: this will produce incorrect results if _state doesn't have the
  /// same set of positions as *this.
  int i = 0;
  for (std::vector<double>::const_iterator iterA =
       this->positions.begin(), iterB = _state.positions.begin();
       iterA != this->positions.end() &&
       iterB != _state.positions.end(); ++iterA, ++iterB, ++i)
  {
    result.positions.push_back((*iterA) + (*iterB));
  }

  return result;
}

/////////////////////////////////////////////////
void JointState::FillSDF(sdf::ElementPtr _sdf)
{
  _sdf->ClearElements();

  _sdf->GetAttribute("name")->Set(this->name);

  int i = 0;
  for (std::vector<double>::const_iterator iter =
       this->positions.begin(); iter != this->positions.end(); ++iter, ++i)
  {
    sdf::ElementPtr elem = _sdf->AddElement("angle");
    elem->GetAttribute("axis")->Set(i);
    elem->Set((*iter));
  }
}
