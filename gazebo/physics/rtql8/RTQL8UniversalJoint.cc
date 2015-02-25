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
/* Desc: A universal joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include "gazebo_config.h"
#include "common/Console.hh"

#include "physics/Link.hh"
#include "physics/rtql8/RTQL8UniversalJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
RTQL8UniversalJoint::RTQL8UniversalJoint(BasePtr _parent)
    : UniversalJoint<RTQL8Joint>(_parent)
{
//   this->jointId = dJointCreateUniversal(_worldId, NULL);
}

//////////////////////////////////////////////////
RTQL8UniversalJoint::~RTQL8UniversalJoint()
{
}

//////////////////////////////////////////////////
math::Vector3 RTQL8UniversalJoint::GetAnchor(int /*index*/) const
{
//   dVector3 result;
//   dJointGetUniversalAnchor(this->jointId, result);
// 
//   return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void RTQL8UniversalJoint::SetAnchor(int /*index*/, const math::Vector3 &_anchor)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
// 
//   dJointSetUniversalAnchor(this->jointId, _anchor.x, _anchor.y, _anchor.z);
}

//////////////////////////////////////////////////
math::Vector3 RTQL8UniversalJoint::GetGlobalAxis(int _index) const
{
//   dVector3 result;
// 
//   if (_index == 0)
//     dJointGetUniversalAxis1(this->jointId, result);
//   else
//     dJointGetUniversalAxis2(this->jointId, result);
// 
//   return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void RTQL8UniversalJoint::SetAxis(int _index, const math::Vector3 &_axis)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
// 
//   if (_index == 0)
//     dJointSetUniversalAxis1(this->jointId, _axis.x, _axis.y, _axis.z);
//   else
//     dJointSetUniversalAxis2(this->jointId, _axis.x, _axis.y, _axis.z);
}

//////////////////////////////////////////////////
void RTQL8UniversalJoint::SetDamping(int /*_index*/, double _damping)
{
//   dJointSetDamping(this->jointId, _damping);
}

//////////////////////////////////////////////////
math::Angle RTQL8UniversalJoint::GetAngleImpl(int _index) const
{
  math::Angle result;

//   if (this->jointId)
//   {
//     if (_index == 0)
//       result = dJointGetUniversalAngle1(this->jointId);
//     else
//       result = dJointGetUniversalAngle2(this->jointId);
//   }

  return result;
}

//////////////////////////////////////////////////
double RTQL8UniversalJoint::GetVelocity(int _index) const
{
  double result;

//   if (_index == 0)
//     result = dJointGetUniversalAngle1Rate(this->jointId);
//   else
//     result = dJointGetUniversalAngle2Rate(this->jointId);

  return result;
}

//////////////////////////////////////////////////
void RTQL8UniversalJoint::SetVelocity(int _index, double _angle)
{
//   if (_index == 0)
//     this->SetParam(dParamVel, _angle);
//   else
//     this->SetParam(dParamVel2, _angle);
}

//////////////////////////////////////////////////
void RTQL8UniversalJoint::SetForce(int _index, double _torque)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
//   if (_index == 0)
//     dJointAddUniversalTorques(this->jointId, _torque, 0);
//   else
//     dJointAddUniversalTorques(this->jointId, 0, _torque);
}

//////////////////////////////////////////////////
void RTQL8UniversalJoint::SetMaxForce(int _index, double _t)
{
//   if (_index == 0)
//     this->SetParam(dParamFMax, _t);
//   else
//     this->SetParam(dParamFMax2, _t);
}

//////////////////////////////////////////////////
double RTQL8UniversalJoint::GetMaxForce(int _index)
{
//   if (_index == 0)
//     return this->GetParam(dParamFMax);
//   else
//     return this->GetParam(dParamFMax2);
  return 0;
}

//////////////////////////////////////////////////
void RTQL8UniversalJoint::SetParam(int _parameter, double _value)
{
//   ODEJoint::SetParam(_parameter, _value);
//   dJointSetUniversalParam(this->jointId, _parameter, _value);
}







