/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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

#include "gazebo/common/Exception.hh"
#include "gazebo/physics/State.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
State::State()
{
  this->wallTime = common::Time::GetWallTime();
}

/////////////////////////////////////////////////
State::State(const std::string &_name, const common::Time &_realTime,
             const common::Time &_simTime, const uint64_t _iterations)
: name(_name), wallTime(common::Time::GetWallTime()), realTime(_realTime),
  simTime(_simTime), iterations(_iterations)
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
common::Time State::GetWallTime() const
{
  return this->wallTime;
}

/////////////////////////////////////////////////
common::Time State::GetRealTime() const
{
  return this->realTime;
}

/////////////////////////////////////////////////
common::Time State::GetSimTime() const
{
  return this->simTime;
}

/////////////////////////////////////////////////
uint64_t State::GetIterations() const
{
  return this->iterations;
}

/////////////////////////////////////////////////
State &State::operator=(const State &_state)
{
  this->name = _state.name;
  this->wallTime = _state.wallTime;
  this->realTime = _state.realTime;
  this->simTime = _state.simTime;
  this->iterations = _state.iterations;

  return *this;
}

/////////////////////////////////////////////////
State State::operator-(const State &_state) const
{
  // Make sure the names match
  if (_state.name != this->name)
  {
    gzthrow("Invalid state substraction operator this[" + this->name +
            "] != [" + _state.name + "]\n");
  }

  return State(this->name, this->realTime - _state.realTime,
               this->simTime - _state.simTime,
               this->iterations - _state.iterations);
}

/////////////////////////////////////////////////
void State::SetWallTime(const common::Time &_time)
{
  this->wallTime = _time;
}

/////////////////////////////////////////////////
void State::SetRealTime(const common::Time &_time)
{
  this->realTime = _time;
}

/////////////////////////////////////////////////
void State::SetSimTime(const common::Time &_time)
{
  this->simTime = _time;
}

/////////////////////////////////////////////////
void State::SetIterations(const uint64_t _iterations)
{
  this->iterations = _iterations;
}
