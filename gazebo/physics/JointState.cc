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
  this->angles.push_back(_joint->GetAngle(0));
}

/////////////////////////////////////////////////
JointState::~JointState()
{
}

/////////////////////////////////////////////////
void JointState::Load(sdf::ElementPtr /*_elem*/)
{
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

  if (_axis < this->angles.size())
    angle = this->angles[_axis];
  else
    gzerr << "Index[" << _axis << " is out of range.\n";

  return angle;
}
