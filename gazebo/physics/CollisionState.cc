/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/CollisionState.hh"

using namespace gazebo;
using namespace physics;

/////////////////////////////////////////////////
CollisionState::CollisionState()
: State()
{
}

/////////////////////////////////////////////////
CollisionState::CollisionState(const CollisionPtr _collision)
: State(_collision->GetName(), _collision->GetWorld()->GetRealTime(),
        _collision->GetWorld()->GetSimTime())
{
  this->pose = _collision->GetRelativePose();
}

/////////////////////////////////////////////////
CollisionState::CollisionState(const sdf::ElementPtr _sdf)
  : State()
{
  // Load the state from SDF
  this->Load(_sdf);
}

/////////////////////////////////////////////////
CollisionState::~CollisionState()
{
}

/////////////////////////////////////////////////
void CollisionState::Load(const sdf::ElementPtr _elem)
{
  // Set the name
  this->name = _elem->Get<std::string>("name");

  // Set the pose
  if (_elem->HasElement("pose"))
    this->pose = _elem->Get<math::Pose>("pose");
  else
    this->pose.Set(0, 0, 0, 0, 0, 0);
}

/////////////////////////////////////////////////
const math::Pose &CollisionState::GetPose() const
{
  return this->pose;
}

/////////////////////////////////////////////////
bool CollisionState::IsZero() const
{
  return this->pose == math::Pose::Zero;
}

/////////////////////////////////////////////////
CollisionState &CollisionState::operator=(const CollisionState &_state)
{
  State::operator=(_state);
  this->pose = _state.pose;
  return *this;
}

/////////////////////////////////////////////////
CollisionState CollisionState::operator-(const CollisionState &_state) const
{
  CollisionState result;
  result.name = this->name;

  // Subtract the pose
  result.pose.pos = this->pose.pos - _state.pose.pos;
  result.pose.rot = _state.pose.rot.GetInverse() * this->pose.rot;

  return result;
}

/////////////////////////////////////////////////
CollisionState CollisionState::operator+(const CollisionState &_state) const
{
  CollisionState result;
  result.name = this->name;

  // Add the pose
  result.pose.pos = this->pose.pos + _state.pose.pos;
  result.pose.rot = _state.pose.rot * this->pose.rot;

  return result;
}

/////////////////////////////////////////////////
void CollisionState::FillSDF(sdf::ElementPtr _sdf)
{
  _sdf->ClearElements();

  _sdf->GetAttribute("name")->Set(this->name);
  _sdf->GetElement("pose")->Set(this->pose);
}

