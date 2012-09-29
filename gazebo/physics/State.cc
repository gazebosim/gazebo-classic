/*
 * Copyright 2011 Nate Koenig
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

#include "State.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
State::State()
{
  this->wallTime = common::Time::GetWallTime();
}

/////////////////////////////////////////////////
State::State(const std::string &_name, const common::Time &_realTime,
             const common::Time &_simTime)
: name(_name), wallTime(common::Time::GetWallTime()), realTime(_realTime),
  simTime(_simTime)
{
}

/////////////////////////////////////////////////
State::~State()
{
}

/////////////////////////////////////////////////
std::string State::GetName() const
{
  return this->name;
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
