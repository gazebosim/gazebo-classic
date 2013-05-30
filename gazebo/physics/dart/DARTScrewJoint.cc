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

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTScrewJoint::DARTScrewJoint(BasePtr _parent)
    : ScrewJoint<DARTJoint>(_parent)
{
}

//////////////////////////////////////////////////
DARTScrewJoint::~DARTScrewJoint()
{
}

//////////////////////////////////////////////////
void DARTScrewJoint::Load(sdf::ElementPtr /*_sdf*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
math::Vector3 DARTScrewJoint::GetGlobalAxis(int /*index*/) const
{
  gzwarn << "Not implemented!\n";

  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Angle DARTScrewJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;

  gzwarn << "Not implemented!\n";

  return result;
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetVelocity(int /*index*/) const
{
  gzwarn << "Not implemented!\n";

  return 0;
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetVelocity(int /*index*/, double /*_angle*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetAxis(int /*index*/, const math::Vector3& /*_axis*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetDamping(int /*index*/, double /*_damping*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetThreadPitch(int /*_index*/, double /*_threadPitch*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::ApplyDamping()
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetForce(int /*index*/, double /*_force*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetParam(int /*_parameter*/, double /*_value*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetParam(int /*_parameter*/) const
{
  gzwarn << "Not implemented!\n";

  return 0;
}

//////////////////////////////////////////////////
void DARTScrewJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzwarn << "Not implemented!\n";
}

//////////////////////////////////////////////////
double DARTScrewJoint::GetMaxForce(int /*_index*/)
{
  gzwarn << "Not implemented!\n";

  return 0;
}





