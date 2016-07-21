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
#include "gazebo/physics/ode/ODEHingeJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEHingeJoint::ODEHingeJoint(dWorldID _worldId, BasePtr _parent)
: HingeJoint<ODEJoint>(_parent)
{
  this->odeJointDPtr->jointId = dJointCreateHinge(_worldId, nullptr);
}

//////////////////////////////////////////////////
ODEHingeJoint::~ODEHingeJoint()
{
  this->jointDPtr->applyDamping.reset();
}

//////////////////////////////////////////////////
void ODEHingeJoint::Load(sdf::ElementPtr _sdf)
{
  HingeJoint<ODEJoint>::Load(_sdf);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEHingeJoint::Anchor(
    const unsigned int /*index*/) const
{
  dVector3 result;

  if (this->odeJointDPtr->jointId)
    dJointGetHingeAnchor(this->odeJointDPtr->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetAnchor(const unsigned int /*index*/,
    const ignition::math::Vector3d &_anchor)
{
  if (this->jointDPtr->childLink)
    this->jointDPtr->childLink->SetEnabled(true);
  if (this->jointDPtr->parentLink)
    this->jointDPtr->parentLink->SetEnabled(true);

  if (this->odeJointDPtr->jointId)
  {
    dJointSetHingeAnchor(this->odeJointDPtr->jointId,
        _anchor.X(), _anchor.Y(), _anchor.Z());
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}


//////////////////////////////////////////////////
ignition::math::Vector3d ODEHingeJoint::GlobalAxis(
    const unsigned int /*_index*/) const
{
  dVector3 result;
  if (this->odeJointDPtr->jointId)
    dJointGetHingeAxis(this->odeJointDPtr->jointId, result);
  else
  {
    gzerr << "ODE Joint ID is invalid\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  ODEJoint::SetAxis(_index, _axis);

  if (this->jointDPtr->childLink)
    this->jointDPtr->childLink->SetEnabled(true);
  if (this->jointDPtr->parentLink)
    this->jointDPtr->parentLink->SetEnabled(true);

  // ODE needs global axis
  ignition::math::Quaterniond axisFrame = this->AxisFrame(0);
  ignition::math::Vector3d globalAxis = axisFrame.RotateVector(_axis);

  if (this->odeJointDPtr->jointId)
  {
    dJointSetHingeAxis(this->odeJointDPtr->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
  }
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
ignition::math::Angle ODEHingeJoint::AngleImpl(
    const unsigned int /*index*/) const
{
  ignition::math::Angle result;
  if (this->odeJointDPtr->jointId)
    result = dJointGetHingeAngle(this->odeJointDPtr->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
double ODEHingeJoint::Velocity(const unsigned int /*index*/) const
{
  double result = 0;

  if (this->odeJointDPtr->jointId)
    result = dJointGetHingeAngleRate(this->odeJointDPtr->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetVelocity(const unsigned int _index, const double _angle)
{
  this->SetVelocityMaximal(_index, _angle);
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetForceImpl(const unsigned int /*_index*/,
    const double _effort)
{
  if (this->odeJointDPtr->jointId)
    dJointAddHingeTorque(this->odeJointDPtr->jointId, _effort);
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEHingeJoint::Param(const unsigned int _parameter) const
{
  double result = 0;

  if (this->odeJointDPtr->jointId)
    result = dJointGetHingeParam(this->odeJointDPtr->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEHingeJoint::SetParam(const unsigned int _parameter, const double _value)
{
  ODEJoint::SetParam(_parameter, _value);

  if (this->odeJointDPtr->jointId)
    dJointSetHingeParam(this->odeJointDPtr->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}
