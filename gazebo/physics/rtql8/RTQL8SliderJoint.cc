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
/* Desc: A slider or primastic joint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */
#include <boost/bind.hpp>

#include "gazebo_config.h"
#include "common/Console.hh"

#include "physics/Link.hh"
#include "physics/rtql8/RTQL8SliderJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
RTQL8SliderJoint::RTQL8SliderJoint(BasePtr _parent)
    : SliderJoint<RTQL8Joint>(_parent)
{
//   this->jointId = dJointCreateSlider(_worldId, NULL);
}

//////////////////////////////////////////////////
RTQL8SliderJoint::~RTQL8SliderJoint()
{
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::Load(sdf::ElementPtr _sdf)
{
//   SliderJoint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
math::Vector3 RTQL8SliderJoint::GetGlobalAxis(int /*_index*/) const
{
//   dVector3 result;
//   dJointGetSliderAxis(this->jointId, result);
//   return math::Vector3(result[0], result[1], result[2]);
  return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Angle RTQL8SliderJoint::GetAngleImpl(int /*_index*/) const
{
   math::Angle result;
//   if (this->jointId)
//     result = dJointGetSliderPosition(this->jointId);
   return result;
}

//////////////////////////////////////////////////
double RTQL8SliderJoint::GetVelocity(int /*index*/) const
{
//   double result = dJointGetSliderPositionRate(this->jointId);
//   return result;
  return 0;
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::SetVelocity(int /*index*/, double _angle)
{
//   this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::SetAxis(int /*index*/, const math::Vector3 &_axis)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink) this->parentLink->SetEnabled(true);
// 
//   dJointSetSliderAxis(this->jointId, _axis.x, _axis.y, _axis.z);
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::SetDamping(int /*index*/, double _damping)
{
//   this->damping_coefficient = _damping;
//   dJointSetDamping(this->jointId, this->damping_coefficient);
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::ApplyDamping()
{
//   double damping_force = this->damping_coefficient * this->GetVelocity(0);
//   this->SetForce(0, damping_force);
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::SetForce(int /*index*/, double _force)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink)
//     this->parentLink->SetEnabled(true);
// 
//   dJointAddSliderForce(this->jointId, _force);
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::SetParam(int _parameter, double _value)
{
//   ODEJoint::SetParam(_parameter, _value);
//   dJointSetSliderParam(this->jointId, _parameter, _value);
}

//////////////////////////////////////////////////
double RTQL8SliderJoint::GetParam(int _parameter) const
{
//   double result = dJointGetSliderParam(this->jointId, _parameter);
//   return result;
  return 0;
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::SetMaxForce(int /*_index*/, double _t)
{
//   this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double RTQL8SliderJoint::GetMaxForce(int /*_index*/)
{
//   return this->GetParam(dParamFMax);

  return 0;
}





