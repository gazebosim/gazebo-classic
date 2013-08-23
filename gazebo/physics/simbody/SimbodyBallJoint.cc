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
/* Desc: A ball joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyBallJoint::SimbodyBallJoint(SimTK::MultibodySystem *_world,
                                   BasePtr _parent)
    : BallJoint<SimbodyJoint>(_parent)
{
}

//////////////////////////////////////////////////
SimbodyBallJoint::~SimbodyBallJoint()
{
}

//////////////////////////////////////////////////
math::Vector3 SimbodyBallJoint::GetAnchor(int /*_index*/) const
{
  return this->anchorPos;
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetAnchor(int /*_index*/,
                                const math::Vector3 &/*_anchor*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetDamping(int /*_index*/, double /*_damping*/)
{
  gzerr << "Not implemented\n";
}

/////////////////////////////////////////////////
void SimbodyBallJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

/////////////////////////////////////////////////
double SimbodyBallJoint::GetVelocity(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
double SimbodyBallJoint::GetMaxForce(int /*_index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
void SimbodyBallJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
  gzerr << "Not implemented\n";
  return;
}

/////////////////////////////////////////////////
math::Angle SimbodyBallJoint::GetAngle(int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
math::Vector3 SimbodyBallJoint::GetGlobalAxis(int /*_index*/) const
{
  return math::Vector3();
}

/////////////////////////////////////////////////
math::Angle SimbodyBallJoint::GetAngleImpl(int /*_index*/) const
{
  return math::Angle();
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetHighStop(int /*_index*/,
                                   const math::Angle &/*_angle*/)
{
  gzerr << "Not implemented\n";
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetLowStop(int /*_index*/,
                                  const math::Angle &/*_angle*/)
{
  gzerr << "Not implemented\n";
}

