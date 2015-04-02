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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/simbody/SimbodyTypes.hh"
#include "gazebo/physics/simbody/SimbodyLink.hh"
#include "gazebo/physics/simbody/SimbodyBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
SimbodyBallJoint::SimbodyBallJoint(SimTK::MultibodySystem * /*_world*/,
                                   BasePtr _parent)
    : BallJoint<SimbodyJoint>(_parent)
{
  this->physicsInitialized = false;
}

//////////////////////////////////////////////////
SimbodyBallJoint::~SimbodyBallJoint()
{
}

//////////////////////////////////////////////////
void SimbodyBallJoint::Load(sdf::ElementPtr _sdf)
{
  BallJoint<SimbodyJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
math::Vector3 SimbodyBallJoint::GetAnchor(unsigned int /*_index*/) const
{
  return this->anchorPos;
}

/////////////////////////////////////////////////
void SimbodyBallJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

/////////////////////////////////////////////////
double SimbodyBallJoint::GetVelocity(unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
double SimbodyBallJoint::GetMaxForce(unsigned int /*_index*/)
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
void SimbodyBallJoint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
  gzerr << "Not implemented\n";
  return;
}

/////////////////////////////////////////////////
math::Vector3 SimbodyBallJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Vector3();
}

/////////////////////////////////////////////////
math::Angle SimbodyBallJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return math::Angle();
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetForceImpl(unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetAxis(unsigned int /*_index*/,
                               const math::Vector3 &/*_axis*/)
{
  gzerr << "SimbodyBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
math::Angle SimbodyBallJoint::GetHighStop(unsigned int /*_index*/)
{
  gzerr << "SimbodyBallJoint::GetHighStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle SimbodyBallJoint::GetLowStop(unsigned int /*_index*/)
{
  gzerr << "SimbodyBallJoint::GetLowStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
bool SimbodyBallJoint::SetHighStop(unsigned int /*_index*/,
                                   const math::Angle &/*_angle*/)
{
  gzerr << "SimbodyBallJoint::SetHighStop not implemented" << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool SimbodyBallJoint::SetLowStop(unsigned int /*_index*/,
                                  const math::Angle &/*_angle*/)
{
  gzerr << "SimbodyBallJoint::SetLowStop not implemented" << std::endl;
  return false;
}

