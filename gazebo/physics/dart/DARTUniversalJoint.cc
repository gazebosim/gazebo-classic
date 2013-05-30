/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTUniversalJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
DARTUniversalJoint::DARTUniversalJoint(BasePtr _parent)
    : UniversalJoint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTUniversalJoint::~DARTUniversalJoint()
{
}

//////////////////////////////////////////////////
math::Vector3 DARTUniversalJoint::GetAnchor(int /*index*/) const
{
  gzwarn << "Not implemented!\n";

  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetAnchor(int /*index*/, const math::Vector3& /*_anchor*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
math::Vector3 DARTUniversalJoint::GetGlobalAxis(int /*_index*/) const
{
  gzwarn << "Not implemented!\n";

  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetAxis(int /*_index*/, const math::Vector3& /*_axis*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetDamping(int /*_index*/, double /*_damping*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
math::Angle DARTUniversalJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;

  gzwarn << "Not implemented!\n";
  return result;
}

//////////////////////////////////////////////////
double DARTUniversalJoint::GetVelocity(int /*_index*/) const
{
  double result = 0;

  gzwarn << "Not implemented!\n";

  return result;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetForce(int /*_index*/, double /*_torque*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
double DARTUniversalJoint::GetMaxForce(int /*_index*/)
{
  gzwarn << "Not implemented!\n";

  return 0;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetParam(int /*_parameter*/, double /*_value*/)
{
  gzwarn << "Not implemented!\n";
}







