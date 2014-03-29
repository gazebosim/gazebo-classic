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
/* Desc: An ODE ball joint
 * Author: Nate Koenig
 * Date: k13 Oct 2009
 */

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/ode/ODEBallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEBallJoint::ODEBallJoint(dWorldID _worldId, BasePtr _parent)
: BallJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateBall(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEBallJoint::~ODEBallJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
math::Vector3 ODEBallJoint::GetAnchor(unsigned int /*_index*/) const
{
  dVector3 result;
  if (this->jointId)
    dJointGetBallAnchor(jointId, result);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////
void ODEBallJoint::SetAnchor(unsigned int /*_index*/,
    const math::Vector3 &_anchor)
{
  if (this->jointId)
    dJointSetBallAnchor(jointId, _anchor.x, _anchor.y, _anchor.z);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEBallJoint::SetForceImpl(unsigned int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}

//////////////////////////////////////////////////
math::Vector3 ODEBallJoint::GetGlobalAxis(unsigned int /*_index*/) const
{
  return math::Vector3();
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
double ODEBallJoint::GetMaxForce(unsigned int /*_index*/)
{
  return 0;
}

//////////////////////////////////////////////////
void ODEBallJoint::SetMaxForce(unsigned int /*_index*/, double /*_t*/)
{
}

//////////////////////////////////////////////////
math::Angle ODEBallJoint::GetAngleImpl(unsigned int /*_index*/) const
{
  return math::Angle(0);
}

//////////////////////////////////////////////////
void ODEBallJoint::SetAxis(unsigned int /*_index*/,
                            const math::Vector3 &/*_axis*/)
{
  gzerr << "ODEBallJoint::SetAxis not implemented" << std::endl;
}

//////////////////////////////////////////////////
math::Angle ODEBallJoint::GetHighStop(unsigned int /*_index*/)
{
  gzerr << "ODEBallJoint::GetHighStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
math::Angle ODEBallJoint::GetLowStop(unsigned int /*_index*/)
{
  gzerr << "ODEBallJoint::GetLowStop not implemented" << std::endl;
  return math::Angle();
}

//////////////////////////////////////////////////
bool ODEBallJoint::SetHighStop(unsigned int /*_index*/,
                               const math::Angle &/*_angle*/)
{
  gzerr << "ODEBallJoint::SetHighStop not implemented" << std::endl;
  return false;
}

//////////////////////////////////////////////////
bool ODEBallJoint::SetLowStop(unsigned int /*_index*/,
                              const math::Angle &/*_angle*/)
{
  gzerr << "ODEBallJoint::SetLowStop not implemented" << std::endl;
  return false;
}
