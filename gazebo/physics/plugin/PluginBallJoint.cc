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
/* Desc: An Plugin ball joint
 * Author: Nate Koenig
 * Date: k13 Oct 2009
 */

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/PhysicsPlugin.h"
#include "gazebo/physics/plugin/PluginBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginBallJoint::PluginBallJoint(BasePtr _parent)
: BallJoint<PluginJoint>(_parent)
{
}

//////////////////////////////////////////////////
PluginBallJoint::~PluginBallJoint()
{
}

//////////////////////////////////////////////////
math::Vector3 PluginBallJoint::GetAnchor(unsigned int /*_index*/) const
{
  double result[3];
  return math::Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////
void PluginBallJoint::SetAnchor(unsigned int /*_index*/,
    const math::Vector3 &_anchor)
{
}

//////////////////////////////////////////////////
void PluginBallJoint::SetForceImpl(unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
math::Vector3 PluginBallJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  return math::Vector3();
}

//////////////////////////////////////////////////
void PluginBallJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
}

//////////////////////////////////////////////////
double PluginBallJoint::GetVelocity(unsigned int /*_index*/) const
{
  return 0;
}

//////////////////////////////////////////////////
double PluginBallJoint::GetMaxForce(unsigned int /*_index*/)
{
  return 0;
}

//////////////////////////////////////////////////
void PluginBallJoint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
}

//////////////////////////////////////////////////
math::Angle PluginBallJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  return math::Angle(0);
}

//////////////////////////////////////////////////
void PluginBallJoint::SetAxis(unsigned int /*_index*/,
                            const math::Vector3 &/*_axis*/)
{
  gzerr << "PluginBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
math::Angle PluginBallJoint::GetHighStop(unsigned int /*_index*/)
{
  gzerr << "PluginBallJoint::GetHighStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle PluginBallJoint::GetLowStop(unsigned int /*_index*/)
{
  gzerr << "PluginBallJoint::GetLowStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
bool PluginBallJoint::SetHighStop(unsigned int /*_index*/,
                               const math::Angle &/*_angle*/)
{
  gzerr << "PluginBallJoint::SetHighStop not implemented" << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool PluginBallJoint::SetLowStop(unsigned int /*_index*/,
                              const math::Angle &/*_angle*/)
{
  gzerr << "PluginBallJoint::SetLowStop not implemented" << std::endl;
  return false;
}
