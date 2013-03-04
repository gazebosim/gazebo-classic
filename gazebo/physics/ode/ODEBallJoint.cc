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
/* Desc: An ODE ball joint
 * Author: Nate Koenig
 * Date: k13 Oct 2009
 */

#include "gazebo_config.h"
#include "common/Console.hh"
#include "physics/ode/ODEBallJoint.hh"

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
math::Vector3 ODEBallJoint::GetAnchor(int /*_index*/) const
{
  dVector3 result;
  dJointGetBallAnchor(jointId, result);
  return math::Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////
void ODEBallJoint::SetAnchor(int /*_index*/, const math::Vector3 &_anchor)
{
  dJointSetBallAnchor(jointId, _anchor.x, _anchor.y, _anchor.z);
}

//////////////////////////////////////////////////
void ODEBallJoint::SetDamping(int /*_index*/, double _damping)
{
  this->dampingCoefficient = _damping;
  // use below when ode version is fixed
  // dJointSetDamping(this->jointId, _damping);
  if (this->useCFMDamping)
    this->applyDamping = physics::Joint::ConnectJointUpdate(
      boost::bind(&ODEJoint::CFMDamping, this));
  else
    this->applyDamping = physics::Joint::ConnectJointUpdate(
      boost::bind(&ODEJoint::ApplyDamping, this));
}
