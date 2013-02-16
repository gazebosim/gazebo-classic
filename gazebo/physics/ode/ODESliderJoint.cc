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
#include "physics/ode/ODESliderJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODESliderJoint::ODESliderJoint(dWorldID _worldId, BasePtr _parent)
    : SliderJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateSlider(_worldId, NULL);
}

//////////////////////////////////////////////////
ODESliderJoint::~ODESliderJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

//////////////////////////////////////////////////
void ODESliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
math::Vector3 ODESliderJoint::GetGlobalAxis(int /*_index*/) const
{
  dVector3 result;
  dJointGetSliderAxis(this->jointId, result);
  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
math::Angle ODESliderJoint::GetAngleImpl(int /*_index*/) const
{
  math::Angle result;
  if (this->jointId)
    result = dJointGetSliderPosition(this->jointId);
  return result;
}

//////////////////////////////////////////////////
double ODESliderJoint::GetVelocity(int /*index*/) const
{
  double result = dJointGetSliderPositionRate(this->jointId);
  return result;
}

//////////////////////////////////////////////////
void ODESliderJoint::SetVelocity(int /*index*/, double _angle)
{
  this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void ODESliderJoint::SetAxis(int /*index*/, const math::Vector3 &_axis)
{
  Joint::SetAxis(0, _axis);

  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink) this->parentLink->SetEnabled(true);

  dJointSetSliderAxis(this->jointId, _axis.x, _axis.y, _axis.z);
}

//////////////////////////////////////////////////
void ODESliderJoint::SetDamping(int /*index*/, double _damping)
{
  this->dampingCoefficient = _damping;
  // use below when ode version is fixed
  // dJointSetDamping(this->jointId, this->dampingCoefficient);
  this->applyDamping = physics::Joint::ConnectJointUpdate(
    boost::bind(&Joint::ApplyDamping, this));
}

//////////////////////////////////////////////////
void ODESliderJoint::SetForce(int _index, double _force)
{
  ODEJoint::SetForce(_index, _force);
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  dJointAddSliderForce(this->jointId, _force);
}

//////////////////////////////////////////////////
void ODESliderJoint::SetParam(int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);
  dJointSetSliderParam(this->jointId, _parameter, _value);
}

//////////////////////////////////////////////////
double ODESliderJoint::GetParam(int _parameter) const
{
  double result = dJointGetSliderParam(this->jointId, _parameter);
  return result;
}

//////////////////////////////////////////////////
void ODESliderJoint::SetMaxForce(int /*_index*/, double _t)
{
  this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double ODESliderJoint::GetMaxForce(int /*_index*/)
{
  return this->GetParam(dParamFMax);
}





