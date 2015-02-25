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

#include <boost/bind.hpp>

#include "gazebo_config.h"
#include "common/Console.hh"

#include "physics/Link.hh"
#include "physics/rtql8/RTQL8HingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8HingeJoint::RTQL8HingeJoint(BasePtr _parent)
    : HingeJoint<RTQL8Joint>(_parent)
{
//   this->jointId = dJointCreateHinge(_worldId, NULL);
}

//////////////////////////////////////////////////
RTQL8HingeJoint::~RTQL8HingeJoint()
{
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::Load(sdf::ElementPtr _sdf)
{
//   HingeJoint<ODEJoint>::Load(_sdf);
// 
//   this->SetParam(dParamFMax, 0);
//   this->SetForce(0, 0);
}

//////////////////////////////////////////////////
math::Vector3 RTQL8HingeJoint::GetAnchor(int /*index*/) const
{
//   dVector3 result;
// 
//   dJointGetHingeAnchor(this->jointId, result);
// 
//   return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetAnchor(int /*index*/, const math::Vector3 &_anchor)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink)
//     this->parentLink->SetEnabled(true);
// 
//   dJointSetHingeAnchor(this->jointId, _anchor.x, _anchor.y, _anchor.z);
}


//////////////////////////////////////////////////
math::Vector3 RTQL8HingeJoint::GetGlobalAxis(int /*_index*/) const
{
//     dVector3 result;
//     dJointGetHingeAxis(this->jointId, result);
//     return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetAxis(int /*index*/, const math::Vector3 &_axis)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink)
//     this->parentLink->SetEnabled(true);
// 
//   dJointSetHingeAxis(this->jointId, _axis.x, _axis.y, _axis.z);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetDamping(int /*index*/, double _damping)
{
//   this->damping_coefficient = _damping;
//   dJointSetDamping(this->jointId, this->damping_coefficient);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::ApplyDamping()
{
//   double damping_force = this->damping_coefficient * this->GetVelocity(0);
//   this->SetForce(0, damping_force);
}

//////////////////////////////////////////////////
math::Angle RTQL8HingeJoint::GetAngleImpl(int /*index*/) const
{
//   math::Angle result;
//   if (this->jointId)
//     result = dJointGetHingeAngle(this->jointId);
//   return result;
  return 0;
}

//////////////////////////////////////////////////
double RTQL8HingeJoint::GetVelocity(int /*index*/) const
{
//   double result = dJointGetHingeAngleRate(this->jointId);
// 
//   return result;
  return 0;
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetVelocity(int /*index*/, double _angle)
{
//   this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetMaxForce(int /*index*/, double _t)
{
//   return this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double RTQL8HingeJoint::GetMaxForce(int /*index*/)
{
//   return this->GetParam(dParamFMax);
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetForce(int /*index*/, double _torque)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink)
//     this->parentLink->SetEnabled(true);
//   dJointAddHingeTorque(this->jointId, _torque);
}

//////////////////////////////////////////////////
double RTQL8HingeJoint::GetParam(int _parameter) const
{
//   double result = dJointGetHingeParam(this->jointId, _parameter);
// 
//   return result;
  return 0;
}

//////////////////////////////////////////////////
void RTQL8HingeJoint::SetParam(int _parameter, double _value)
{
//   ODEJoint::SetParam(_parameter, _value);
// 
//   dJointSetHingeParam(this->jointId, _parameter, _value);
}
