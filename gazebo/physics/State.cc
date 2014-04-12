/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "ignition/common/Exception.hh"
#include "gazebo/physics/State.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
State::State()
{
  this->wallTime = ignition::common::Time::GetWallTime();
}

/////////////////////////////////////////////////
State::State(const std::string &_name, const ignition::common::Time &_realTime,
             const ignition::common::Time &_simTime)
: name(_name),
  wallTime(ignition::common::Time::GetWallTime()), realTime(_realTime),
  simTime(_simTime)
{
}

/////////////////////////////////////////////////
State::~State()
{
}

/////////////////////////////////////////////////
void State::Load(const sdf::ElementPtr /*_elem*/)
{
}

/////////////////////////////////////////////////
std::string State::GetName() const
{
  return this->name;
}

/////////////////////////////////////////////////
void State::SetName(const std::string &_name)
{
  this->name = _name;
}

/////////////////////////////////////////////////
ignition::common::Time State::GetWallTime() const
{
  return this->wallTime;
}

/////////////////////////////////////////////////
ignition::common::Time State::GetRealTime() const
{
  return this->realTime;
}

/////////////////////////////////////////////////
ignition::common::Time State::GetSimTime() const
{
  return this->simTime;
}

/////////////////////////////////////////////////
State &State::operator=(const State &_state)
{
  this->name = _state.name;
  this->wallTime = _state.wallTime;
  this->realTime = _state.realTime;
  this->simTime = _state.simTime;

  return *this;
}

/////////////////////////////////////////////////
State State::operator-(const State &_state) const
{
  // Make sure the names match
  if (_state.name != this->name)
  {
    ignthrow("Invalid state substraction operator this[" + this->name +
            "] != [" + _state.name + "]\n");
  }

  return State(this->name, this->realTime - _state.realTime,
               this->simTime - _state.simTime);
}

/////////////////////////////////////////////////
void State::SetWallTime(const ignition::common::Time &_time)
{
  this->wallTime = _time;
}

/////////////////////////////////////////////////
void State::SetRealTime(const ignition::common::Time &_time)
{
  this->realTime = _time;
}

/////////////////////////////////////////////////
void State::SetSimTime(const ignition::common::Time &_time)
{
  this->simTime = _time;
}
