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
#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/ode/ODEJointPrivate.hh"
#include "gazebo/physics/ode/ODEBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEBallJoint::ODEBallJoint(dWorldID _worldId, BasePtr _parent)
: BallJoint<ODEJoint>(_parent)
{
  this->odeJointDPtr->jointId = dJointCreateBall(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEBallJoint::~ODEBallJoint()
{
  if (this->jointDPtr->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->jointDPtr->applyDamping);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEBallJoint::Anchor(
    const unsigned int /*_index*/) const
{
  dVector3 result;
  if (this->odeJointDPtr->jointId)
    dJointGetBallAnchor(this->odeJointDPtr->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return math::Vector3::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEBallJoint::SetAnchor(const unsigned int /*_index*/,
    const ignition::math::Vector3d &_anchor)
{
  if (this->odeJointDPtr->jointId)
  {
    dJointSetBallAnchor(this->odeJointDPtr->jointId,
        _anchor.X(), _anchor.Y(), _anchor.Z());
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEBallJoint::SetForceImpl(unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEBallJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void ODEBallJoint::SetVelocity(const unsigned int /*_index*/,
    const double /*_angle*/)
{
}

//////////////////////////////////////////////////
double ODEBallJoint::Velocity(const unsigned int /*_index*/) const
{
  return 0;
}

//////////////////////////////////////////////////
ignition::math::Angle ODEBallJoint::AngleImpl(
    const unsigned int /*_index*/) const
{
  return ignition::math::Angle(0);
}

//////////////////////////////////////////////////
void ODEBallJoint::SetAxis(const unsigned int /*_index*/,
                           const ignition::math::Vector3d &/*_axis*/)
{
  gzerr << "ODEBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
ignition::math::Angle ODEBallJoint::HighStop(
    const unsigned int /*_index*/) const
{
  gzerr << "ODEBallJoint::GetHighStop not implemented" << std::endl;
  return ignition::math::Angle();
}

//////////////////////////////////////////////////
ignition::math::Angle ODEBallJoint::LowStop(
    const unsigned int /*_index*/) const
{
  gzerr << "ODEBallJoint::GetLowStop not implemented" << std::endl;
  return ignition::math::Angle();
}

//////////////////////////////////////////////////
bool ODEBallJoint::SetHighStop(const unsigned int /*_index*/,
                               const ignition::math::Angle &/*_angle*/)
{
  gzerr << "ODEBallJoint::SetHighStop not implemented" << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool ODEBallJoint::SetLowStop(const unsigned int /*_index*/,
                              const ignition::math::Angle &/*_angle*/)
{
  gzerr << "ODEBallJoint::SetLowStop not implemented" << std::endl;
  return false;
}
