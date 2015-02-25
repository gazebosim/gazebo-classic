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
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"

#include "gazebo/physics/Link.hh"
#include "gazebo/physics/rtql8/RTQL8SliderJoint.hh"

#include "rtql8/kinematics/Dof.h"
#include "rtql8/kinematics/Joint.h"

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
  //
  SliderJoint<RTQL8Joint>::Load(_sdf);

  // Slider joint has only one degree of freedom.
  // rtql8::kinematics::Dof* dofs = new rtql8::kinematics::Dof;

  // TODO: Could I know the sliding axis?; x, y, z

  //
//  kinematics::TrfmTranslate* trans
//      = new kinematics::TrfmTranslate(dofs[0], dofs[1], dofs[2]);

  //
  //this->rtql8Joint->addTransform(trans);
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
void RTQL8SliderJoint::SetVelocity(int /*index*/, double /*_angle*/)
{
   //this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::SetAxis(int /*index*/, const math::Vector3 &_axis)
{
  // TODO: check whether below code is needed.
   if (this->childLink)
     this->childLink->SetEnabled(true);
   if (this->parentLink)
     this->parentLink->SetEnabled(true);

  // Slider joint has only one degree of freedom.
  // _axis must have a value of (1, 0, 0), (0, 1, 0), and (0, 0, 1)
  if (_axis == math::Vector3(1, 0, 0))
  {
    // When rtql8's 'Joint' is destroied, it deletes all 'Transform's.
    // When 'Transform' is destroied, it deletes all 'Dof's.
    rtql8::kinematics::Dof* dofs = new rtql8::kinematics::Dof;
    rtql8::kinematics::TrfmTranslateX* trans = new rtql8::kinematics::TrfmTranslateX(dofs);
    this->rtql8Joint->addTransform(trans);
  }
  else if (_axis == math::Vector3(1, 0, 0))
  {
    // When rtql8's 'Joint' is destroied, it deletes all 'Transform's.
    // When 'Transform' is destroied, it deletes all 'Dof's.
    rtql8::kinematics::Dof* dofs = new rtql8::kinematics::Dof;
    rtql8::kinematics::TrfmTranslateY* trans = new rtql8::kinematics::TrfmTranslateY(dofs);
    this->rtql8Joint->addTransform(trans);
  }
  else if (_axis == math::Vector3(1, 0, 0))
  {
    // When rtql8's 'Joint' is destroied, it deletes all 'Transform's.
    // When 'Transform' is destroied, it deletes all 'Dof's.
    rtql8::kinematics::Dof* dofs = new rtql8::kinematics::Dof;
    rtql8::kinematics::TrfmTranslateZ* trans = new rtql8::kinematics::TrfmTranslateZ(dofs);
    this->rtql8Joint->addTransform(trans);
  }
  else
  {
    // We assume that the axis has the value among these:
    // (1, 0, 0), (0, 1, 0), (0, 0, 1)
    gzthrow("Axis must be one of these: (1, 0, 0), (0, 1, 0), (0, 0, 1)\n");
  }
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::SetDamping(int /*index*/, double /*_damping*/)
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
void RTQL8SliderJoint::SetForce(int /*index*/, double /*_force*/)
{
//   if (this->childLink)
//     this->childLink->SetEnabled(true);
//   if (this->parentLink)
//     this->parentLink->SetEnabled(true);
// 
//   dJointAddSliderForce(this->jointId, _force);
}

//////////////////////////////////////////////////
void RTQL8SliderJoint::SetMaxForce(int /*_index*/, double /*_t*/)
{
//   this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double RTQL8SliderJoint::GetMaxForce(int /*_index*/)
{
//   return this->GetParam(dParamFMax);

  return 0;
}





