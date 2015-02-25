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

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/rtql8/RTQL8BallJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8BallJoint::RTQL8BallJoint(BasePtr _parent)
: BallJoint<RTQL8Joint>(_parent)
{
  //this->jointId = dJointCreateBall(_worldId, NULL);
}

//////////////////////////////////////////////////
RTQL8BallJoint::~RTQL8BallJoint()
{
}

//////////////////////////////////////////////////
math::Vector3 RTQL8BallJoint::GetAnchor(int /*_index*/) const
{
  //dVector3 result;
  //dJointGetBallAnchor(jointId, result);
  //return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}


//////////////////////////////////////////////////
void RTQL8BallJoint::SetAnchor(int /*_index*/, const math::Vector3 &/*_anchor*/)
{
  //dJointSetBallAnchor(jointId, _anchor.x, _anchor.y, _anchor.z);
}

//////////////////////////////////////////////////
void RTQL8BallJoint::SetDamping(int /*_index*/, double /*_damping*/)
{
  //dJointSetDamping(this->jointId, _damping);
}
