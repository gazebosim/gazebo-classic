/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/simbody/SimbodyJointPrivate.hh"
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
  this->simbodyJointDPtr->physicsInitialized = false;
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
ignition::math::Vector3d SimbodyBallJoint::Anchor(
    unsigned int /*_index*/) const
{
  return this->simbodyJointDPtr->anchorPos;
}

/////////////////////////////////////////////////
void SimbodyBallJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
  gzerr << "Not implemented\n";
}

/////////////////////////////////////////////////
double SimbodyBallJoint::Velocity(unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return 0;
}

/////////////////////////////////////////////////
ignition::math::Vector3d SimbodyBallJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return ignition::math::Vector3d::Zero;
}

/////////////////////////////////////////////////
ignition::math::Angle SimbodyBallJoint::AngleImpl(
    const unsigned int /*_index*/) const
{
  gzerr << "Not implemented\n";
  return ignition::math::Angle::Zero;
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetForceImpl(
    const unsigned int /*_index*/, const double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
void SimbodyBallJoint::SetAxis(unsigned int /*_index*/,
                               const ignition::math::Vector3d &/*_axis*/)
{
  gzerr << "SimbodyBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyBallJoint::HighStop(
    const unsigned int /*_index*/) const
{
  gzerr << "SimbodyBallJoint::HighStop not implemented" << std::endl;
  return ignition::math::Angle::Zero;
}

//////////////////////////////////////////////////
ignition::math::Angle SimbodyBallJoint::LowStop(
    const unsigned int /*_index*/) const
{
  gzerr << "SimbodyBallJoint::LowStop not implemented" << std::endl;
  return ignition::math::Angle::Zero;
}

//////////////////////////////////////////////////
bool SimbodyBallJoint::SetHighStop(const unsigned int /*_index*/,
                                   const ignition::math::Angle &/*_angle*/)
{
  gzerr << "SimbodyBallJoint::SetHighStop not implemented" << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool SimbodyBallJoint::SetLowStop(const unsigned int /*_index*/,
                                  const ignition::math::Angle &/*_angle*/)
{
  gzerr << "SimbodyBallJoint::SetLowStop not implemented" << std::endl;
  return false;
}

//////////////////////////////////////////////////
ignition::math::Vector3d SimbodyBallJoint::Axis(
    const unsigned int /*_index*/) const
{
  return ignition::math::Vector3d::Zero;
}

