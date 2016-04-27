/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODEJointPrivate.hh"
#include "gazebo/physics/ode/ODESliderJoint.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
ODESliderJoint::ODESliderJoint(dWorldID _worldId, BasePtr _parent)
: SliderJoint<ODEJoint>(_parent)
{
  this->odeJointDPtr->jointId = dJointCreateSlider(_worldId, NULL);
}

//////////////////////////////////////////////////
ODESliderJoint::~ODESliderJoint()
{
  if (this->odeJointDPtr->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->odeJointDPtr->applyDamping);
}

//////////////////////////////////////////////////
void ODESliderJoint::Load(sdf::ElementPtr _sdf)
{
  SliderJoint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODESliderJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  dVector3 result;
  if (this->odeJointDPtr->jointId)
    dJointGetSliderAxis(this->odeJointDPtr->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
ignition::math::Angle ODESliderJoint::AngleImpl(
    const unsigned int /*_index*/) const
{
  ignition::math::Angle result;
  if (this->odeJointDPtr->jointId)
    result = dJointGetSliderPosition(this->odeJointDPtr->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODESliderJoint::Velocity(const unsigned int /*index*/) const
{
  double result = 0;
  if (this->odeJointDPtr->jointId)
    result = dJointGetSliderPositionRate(this->odeJointDPtr->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODESliderJoint::SetVelocity(const unsigned int _index,
    const double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
void ODESliderJoint::SetAxis(const unsigned int /*index*/,
    const ignition::math::Vector3d &_axis)
{
  if (this->odeJointDPtr->childLink)
    this->odeJointDPtr->childLink->SetEnabled(true);
  if (this->odeJointDPtr->parentLink)
    this->odeJointDPtr->parentLink->SetEnabled(true);

  // ODE needs global axis
  ignition::math::Quaterniond axisFrame = this->AxisFrame(0);
  ignition::math::Vector3d globalAxis = axisFrame.RotateVector(_axis);

  if (this->odeJointDPtr->jointId)
  {
    dJointSetSliderAxis(this->odeJointDPtr->jointId,
                        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODESliderJoint::SetForceImpl(const unsigned int /*_index*/,
    const double _effort)
{
  if (this->odeJointDPtr->jointId)
    dJointAddSliderForce(this->odeJointDPtr->jointId, _effort);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
void ODESliderJoint::SetParam(const unsigned int _parameter,
    const double _value)
{
  ODEJoint::SetParam(_parameter, _value);
  dJointSetSliderParam(this->odeJointDPtr->jointId, _parameter, _value);
}

//////////////////////////////////////////////////
double ODESliderJoint::Param(const unsigned int _parameter) const
{
  double result = 0;

  if (this->odeJointDPtr->jointId)
    result = dJointGetSliderParam(this->odeJointDPtr->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODESliderJoint::Anchor(
    const unsigned int /*_index*/) const
{
  gzlog << "ODESliderJoint::GetAnchor not implemented.\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void ODESliderJoint::SetAnchor(const unsigned int /*_index*/,
  const ignition::math::Vector3d &/*_anchor*/)
{
  gzlog << "ODESliderJoint::SetAnchor not implemented.\n";
}
