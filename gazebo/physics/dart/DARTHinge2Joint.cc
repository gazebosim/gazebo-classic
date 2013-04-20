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
#include "gazebo/physics/dart/DARTHinge2Joint.hh"

#include "dart/kinematics/Dof.h"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
DARTHinge2Joint::DARTHinge2Joint(BasePtr _parent)
    : Hinge2Joint<DARTJoint>(_parent)
{
  //this->jointId = dJointCreateHinge2(_worldId, NULL);
}

//////////////////////////////////////////////////
DARTHinge2Joint::~DARTHinge2Joint()
{
}

//////////////////////////////////////////////////
void DARTHinge2Joint::Load(sdf::ElementPtr _sdf)
{
  Hinge2Joint<DARTJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
math::Vector3 DARTHinge2Joint::GetAnchor(int /*_index*/) const
{
//   dVector3 result;
// 
//   if (_index == 0)
//     dJointGetHinge2Anchor(this->jointId, result);
//   else
//     dJointGetHinge2Anchor2(this->jointId, result);
// 
//   return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetAnchor(int /*index*/, const math::Vector3 &/*_anchor*/)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
//   dJointSetHinge2Anchor(this->jointId, _anchor.x, _anchor.y, _anchor.z);
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetAxis(int /*_index*/, const math::Vector3 &/*_axis*/)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
// 
//   if (_index == 0)
//     dJointSetHinge2Axis1(this->jointId, _axis.x, _axis.y, _axis.z);
//   else
//     dJointSetHinge2Axis2(this->jointId, _axis.x, _axis.y, _axis.z);
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetDamping(int /*_index*/, double /*_damping*/)
{
//   dJointSetDamping(this->jointId, _damping);
}

//////////////////////////////////////////////////
math::Vector3 DARTHinge2Joint::GetGlobalAxis(int /*_index*/) const
{
//   dVector3 result;
// 
//   if (_index == 0)
//     dJointGetHinge2Axis1(this->jointId, result);
//   else
//     dJointGetHinge2Axis2(this->jointId, result);
// 
//   return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Angle DARTHinge2Joint::GetAngleImpl(int _index) const
{
  math::Angle result;

  if (_index == 0)
    result = this->dartJoint->getDof(0)->getValue();
  else
    result = this->dartJoint->getDof(1)->getValue();

  return result;
}

//////////////////////////////////////////////////
double DARTHinge2Joint::GetVelocity(int /*_index*/) const
{
  double result = 0;

//   if (_index == 0)
//     result = dJointGetHinge2Angle1Rate(this->jointId);
//   else
//     result = dJointGetHinge2Angle2Rate(this->jointId);

  return result;
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetVelocity(int /*_index*/, double /*_angle*/)
{
//   if (_index == 0)
//     this->SetParam(dParamVel, _angle);
//   else
//     this->SetParam(dParamVel2, _angle);
}

//////////////////////////////////////////////////
double DARTHinge2Joint::GetMaxForce(int /*_index*/)
{
//   if (_index == 0)
//     return this->GetParam(dParamFMax);
//   else
//     return this->GetParam(dParamFMax2);
  return 0;
}


//////////////////////////////////////////////////
void DARTHinge2Joint::SetMaxForce(int /*_index*/, double /*_t*/)
{
//   if (_index == 0)
//     this->SetParam(dParamFMax, _t);
//   else
//     this->SetParam(dParamFMax2, _t);
}


//////////////////////////////////////////////////
void DARTHinge2Joint::SetForce(int /*_index*/, double /*_torque*/)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
// 
//   if (_index == 0)
//     dJointAddHinge2Torques(this->jointId, _torque, 0);
//   else
//     dJointAddHinge2Torques(this->jointId, 0, _torque);
}

//////////////////////////////////////////////////
double DARTHinge2Joint::GetParam(int /*_parameter*/) const
{
//   double result = dJointGetHinge2Param(this->jointId, _parameter);
// 
//   return result;
  return 0;
}

//////////////////////////////////////////////////
void DARTHinge2Joint::SetParam(int /*_parameter*/, double /*_value*/)
{
//   ODEJoint::SetParam(_parameter, _value);
//   dJointSetHinge2Param(this->jointId, _parameter, _value);
}
