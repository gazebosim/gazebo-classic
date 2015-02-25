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

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/rtql8/RTQL8ScrewJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
RTQL8ScrewJoint::RTQL8ScrewJoint(BasePtr _parent)
    : ScrewJoint<RTQL8Joint>(_parent)
{
//   this->jointId = dJointCreateScrew(_worldId, NULL);
}

//////////////////////////////////////////////////
RTQL8ScrewJoint::~RTQL8ScrewJoint()
{
}

//////////////////////////////////////////////////
void RTQL8ScrewJoint::Load(sdf::ElementPtr /*_sdf*/)
{
//   ScrewJoint<ODEJoint>::Load(_sdf);
//   this->SetThreadPitch(0, this->threadPitch);
}

//////////////////////////////////////////////////
math::Vector3 RTQL8ScrewJoint::GetGlobalAxis(int /*index*/) const
{
//   dVector3 result;
//   dJointGetScrewAxis(this->jointId, result);
// 
//   return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Angle RTQL8ScrewJoint::GetAngleImpl(int /*_index*/) const
{
//   math::Angle result;
//   if (this->jointId)
//     result = dJointGetScrewPosition(this->jointId);
// 
//   return result;

  return 0;
}

//////////////////////////////////////////////////
double RTQL8ScrewJoint::GetVelocity(int /*index*/) const
{
//   double result = dJointGetScrewPositionRate(this->jointId);
// 
//   return result;

  return 0;
}

//////////////////////////////////////////////////
void RTQL8ScrewJoint::SetVelocity(int /*index*/, double /*_angle*/)
{
//   this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void RTQL8ScrewJoint::SetAxis(int /*index*/, const math::Vector3 &/*_axis*/)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
// 
//   dJointSetScrewAxis(this->jointId, _axis.x, _axis.y, _axis.z);
}

//////////////////////////////////////////////////
void RTQL8ScrewJoint::SetDamping(int /*index*/, double /*_damping*/)
{
//   this->damping_coefficient = _damping;
//   dJointSetDamping(this->jointId, this->damping_coefficient);
}

//////////////////////////////////////////////////
void RTQL8ScrewJoint::SetThreadPitch(int /*_index*/, double /*_threadPitch*/)
{
//   dJointSetScrewThreadPitch(this->jointId, _threadPitch);
}

//////////////////////////////////////////////////
void RTQL8ScrewJoint::ApplyDamping()
{
//   double damping_force = this->damping_coefficient * this->GetVelocity(0);
//   this->SetForce(0, damping_force);
}

//////////////////////////////////////////////////
void RTQL8ScrewJoint::SetForce(int /*index*/, double /*_force*/)
{
//   if (this->childLink) this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
//   // dJointAddScrewForce(this->jointId, _force);
//   dJointAddScrewTorque(this->jointId, _force);
}

//////////////////////////////////////////////////
void RTQL8ScrewJoint::SetParam(int /*_parameter*/, double /*_value*/)
{
//   ODEJoint::SetParam(_parameter, _value);
//   dJointSetScrewParam(this->jointId, _parameter, _value);
}

//////////////////////////////////////////////////
double RTQL8ScrewJoint::GetParam(int /*_parameter*/) const
{
//   double result = dJointGetScrewParam(this->jointId, _parameter);
// 
//   return result;

  return 0;
}

//////////////////////////////////////////////////
void RTQL8ScrewJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
//   this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double RTQL8ScrewJoint::GetMaxForce(int /*_index*/)
{
//   return this->GetParam(dParamFMax);

  return 0;
}





