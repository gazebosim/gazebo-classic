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

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/dart/DARTUniversalJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
DARTUniversalJoint::DARTUniversalJoint(BasePtr _parent)
    : UniversalJoint<DARTJoint>(_parent)
{
//   this->jointId = dJointCreateUniversal(_worldId, NULL);
}

//////////////////////////////////////////////////
DARTUniversalJoint::~DARTUniversalJoint()
{
}

//////////////////////////////////////////////////
math::Vector3 DARTUniversalJoint::GetAnchor(int /*index*/) const
{
//   dVector3 result;
//   dJointGetUniversalAnchor(this->jointId, result);
// 
//   return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetAnchor(int /*index*/, const math::Vector3& /*_anchor*/)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
// 
//   dJointSetUniversalAnchor(this->jointId, _anchor.x, _anchor.y, _anchor.z);
}

//////////////////////////////////////////////////
math::Vector3 DARTUniversalJoint::GetGlobalAxis(int /*_index*/) const
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
void DARTUniversalJoint::SetAxis(int /*_index*/, const math::Vector3& /*_axis*/)
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
void DARTUniversalJoint::SetDamping(int /*_index*/, double /*_damping*/)
{
//   dJointSetDamping(this->jointId, _damping);
}

//////////////////////////////////////////////////
math::Angle DARTUniversalJoint::GetAngleImpl(int /*_index*/) const
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
double DARTUniversalJoint::GetVelocity(int /*_index*/) const
{
  double result = 0;

//   if (_index == 0)
//     result = dJointGetUniversalAngle1Rate(this->jointId);
//   else
//     result = dJointGetUniversalAngle2Rate(this->jointId);

  return result;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetVelocity(int /*_index*/, double /*_angle*/)
{
//   if (_index == 0)
//     this->SetParam(dParamVel, _angle);
//   else
//     this->SetParam(dParamVel2, _angle);
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetForce(int /*_index*/, double /*_torque*/)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
//   if (_index == 0)
//     dJointAddUniversalTorques(this->jointId, _torque, 0);
//   else
//     dJointAddUniversalTorques(this->jointId, 0, _torque);
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
//   if (_index == 0)
//     this->SetParam(dParamFMax, _t);
//   else
//     this->SetParam(dParamFMax2, _t);
}

//////////////////////////////////////////////////
double DARTUniversalJoint::GetMaxForce(int /*_index*/)
{
//   if (_index == 0)
//     return this->GetParam(dParamFMax);
//   else
//     return this->GetParam(dParamFMax2);
  return 0;
}

//////////////////////////////////////////////////
void DARTUniversalJoint::SetParam(int /*_parameter*/, double /*_value*/)
{
//   ODEJoint::SetParam(_parameter, _value);
//   dJointSetUniversalParam(this->jointId, _parameter, _value);
}







