/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
/* Desc: An ODE ball joint
 * Author: Nate Koenig
 * Date: k13 Oct 2009
 */

#include <ignition/math/Helpers.hh>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/ode/ODEBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEBallJoint::ODEBallJoint(dWorldID _worldId, BasePtr _parent)
: BallJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateBall(_worldId, nullptr);
}

//////////////////////////////////////////////////
ODEBallJoint::~ODEBallJoint()
{
  this->applyDamping.reset();
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEBallJoint::Anchor(
    const unsigned int /*_index*/) const
{
  dVector3 result;
  if (this->jointId)
    dJointGetBallAnchor(jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////
void ODEBallJoint::SetAnchor(const unsigned int /*_index*/,
    const ignition::math::Vector3d &_anchor)
{
  if (this->jointId)
    dJointSetBallAnchor(jointId, _anchor.X(), _anchor.Y(), _anchor.Z());
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
  return ignition::math::Vector3d();
}

//////////////////////////////////////////////////
void ODEBallJoint::SetVelocity(unsigned int /*_index*/, double /*_angle*/)
{
}

//////////////////////////////////////////////////
double ODEBallJoint::GetVelocity(unsigned int /*_index*/) const
{
  return 0;
}

//////////////////////////////////////////////////
double ODEBallJoint::PositionImpl(const unsigned int /*_index*/) const
{
  static bool notPrintedYet = true;
  if (notPrintedYet)
  {
    notPrintedYet = false;
    gzerr << "ODEBallJoint::PositionImpl not implemented" << std::endl;
  }
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void ODEBallJoint::SetAxis(const unsigned int /*_index*/,
                            const ignition::math::Vector3d &/*_axis*/)
{
  gzerr << "ODEBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
double ODEBallJoint::UpperLimit(const unsigned int /*_index*/) const
{
  gzerr << "ODEBallJoint::UpperLimit not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
double ODEBallJoint::LowerLimit(const unsigned int /*_index*/) const
{
  gzerr << "ODEBallJoint::LowerLimit not implemented" << std::endl;
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
void ODEBallJoint::SetUpperLimit(const unsigned int /*_index*/,
                                 const double /*_limit*/)
{
  gzerr << "ODEBallJoint::SetUpperLimit not implemented" << std::endl;
}

//////////////////////////////////////////////////
void ODEBallJoint::SetLowerLimit(const unsigned int /*_index*/,
                                 const double /*_limit*/)
{
  gzerr << "ODEBallJoint::SetLowerLimit not implemented" << std::endl;
}
