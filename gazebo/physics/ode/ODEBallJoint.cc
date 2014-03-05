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
math::Vector3 ODEBallJoint::GetAnchor(int /*_index*/) const
{
  dVector3 result;
  if (this->jointId)
    dJointGetBallAnchor(jointId, result);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return math::Vector3(result[0], result[1], result[2]);
}


//////////////////////////////////////////////////
void ODEBallJoint::SetAnchor(int /*_index*/, const math::Vector3 &_anchor)
{
  if (this->jointId)
    dJointSetBallAnchor(jointId, _anchor.x, _anchor.y, _anchor.z);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODEBallJoint::SetForceImpl(int /*_index*/, double /*_torque*/)
{
  gzerr << "Not implemented";
}
