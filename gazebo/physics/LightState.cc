/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#include "gazebo/physics/Light.hh"
#include "gazebo/physics/LightState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
LightState::LightState(const LightPtr _light, const common::Time &_realTime,
    const common::Time &_simTime, const uint64_t _iterations)
: State(_light->GetName(), _realTime, _simTime, _iterations)
{
  this->pose = _light->GetWorldPose().Ign();
}

/////////////////////////////////////////////////
LightState::LightState(const sdf::ElementPtr _sdf)
  : State()
{
  this->Load(_sdf);
}

/////////////////////////////////////////////////
void LightState::Load(const sdf::ElementPtr _elem)
{
  // Set the name
  this->name = _elem->Get<std::string>("name");

  // Set the light pose
  if (_elem->HasElement("pose"))
    this->pose = _elem->Get<ignition::math::Pose3d>("pose");
  else
    this->pose.Set(0, 0, 0, 0, 0, 0);
}

/////////////////////////////////////////////////
void LightState::Load(const LightPtr _light, const common::Time &_realTime,
    const common::Time &_simTime, const uint64_t _iterations)
{
  this->name = _light->GetName();
  this->wallTime = common::Time::GetWallTime();
  this->realTime = _realTime;
  this->simTime = _simTime;
  this->iterations = _iterations;
  this->pose = _light->GetWorldPose().Ign();
}

/////////////////////////////////////////////////
const ignition::math::Pose3d LightState::Pose() const
{
  return this->pose;
}

/////////////////////////////////////////////////
bool LightState::IsZero() const
{
  bool result = true;

  return result && this->pose == ignition::math::Pose3d::Zero;
}

/////////////////////////////////////////////////
LightState &LightState::operator=(const LightState &_state)
{
  State::operator=(_state);

  // Copy the pose
  this->pose = _state.pose;

  return *this;
}

/////////////////////////////////////////////////
LightState LightState::operator-(const LightState &_state) const
{
  LightState result;

  result.name = this->name;
  result.pose.Pos() = this->pose.Pos() - _state.pose.Pos();
  result.pose.Rot() = _state.pose.Rot().Inverse() * this->pose.Rot();

  return result;
}

/////////////////////////////////////////////////
LightState LightState::operator+(const LightState &_state) const
{
  LightState result;

  result.name = this->name;
  result.pose.Pos() = this->pose.Pos() + _state.pose.Pos();
  result.pose.Rot() = _state.pose.Rot() * this->pose.Rot();

  return result;
}

/////////////////////////////////////////////////
void LightState::FillSDF(sdf::ElementPtr _sdf)
{
  _sdf->ClearElements();

  _sdf->GetAttribute("name")->Set(this->name);
  _sdf->GetElement("pose")->Set(this->pose);
}

