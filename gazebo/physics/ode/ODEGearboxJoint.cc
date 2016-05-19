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
#include "gazebo/physics/ode/ODELink.hh"
#include "gazebo/physics/ode/ODEJointPrivate.hh"
#include "gazebo/physics/ode/ODEGearboxJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEGearboxJoint::ODEGearboxJoint(dWorldID _worldId, BasePtr _parent)
: GearboxJoint<ODEJoint>(_parent)
{
  this->odeJointDPtr->jointId = dJointCreateGearbox(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEGearboxJoint::~ODEGearboxJoint()
{
}

//////////////////////////////////////////////////
void ODEGearboxJoint::Init()
{
  Joint::Init();
  LinkPtr link = this->jointDPtr->model->LinkByName(this->referenceBody);
  this->SetReferenceBody(link);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::Load(sdf::ElementPtr _sdf)
{
  GearboxJoint<ODEJoint>::Load(_sdf);

  this->SetGearboxRatio(this->gearRatio);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetReferenceBody(LinkPtr _body)
{
  ODELinkPtr odelink = std::dynamic_pointer_cast<ODELink>(_body);
  dBodyID refId;
  if (odelink == NULL)
  {
    gzwarn << "Reference body not valid, using inertial frame.\n";
    refId = 0;
  }
  else
  {
    refId = odelink->ODEId();
  }

  dJointSetGearboxReferenceBody(this->odeJointDPtr->jointId, refId);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetGearboxRatio(const double _gearRatio)
{
  this->gearRatio = _gearRatio;
  dJointSetGearboxRatio(this->odeJointDPtr->jointId, _gearRatio);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEGearboxJoint::GlobalAxis(
    const unsigned int _index) const
{
  dVector3 result;

  if (_index == 0)
    dJointGetGearboxAxis1(this->odeJointDPtr->jointId, result);
  else if (_index == 1)
    dJointGetGearboxAxis2(this->odeJointDPtr->jointId, result);
  else
    gzerr << "index [" << _index << "] out of range\n";

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  ODEJoint::SetAxis(_index, _axis);

  if (this->jointDPtr->childLink)
    this->jointDPtr->childLink->SetEnabled(true);
  if (this->jointDPtr->parentLink)
    this->jointDPtr->parentLink->SetEnabled(true);

  /// ODE needs global axis
  ignition::math::Quaterniond axisFrame = this->AxisFrame(_index);
  ignition::math::Vector3d globalAxis = axisFrame.RotateVector(_axis);

  if (_index == 0)
  {
    dJointSetGearboxAxis1(this->odeJointDPtr->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
  }
  else if (_index == 1)
  {
    dJointSetGearboxAxis2(this->odeJointDPtr->jointId,
        globalAxis.X(), globalAxis.Y(), globalAxis.Z());
  }
  else
    gzerr << "index [" << _index << "] out of range\n";
}

//////////////////////////////////////////////////
ignition::math::Angle ODEGearboxJoint::AngleImpl(
    const unsigned int /*index*/) const
{
  gzlog << "GetAngle not implemented for gearbox\n";
  return ignition::math::Angle();
}

//////////////////////////////////////////////////
double ODEGearboxJoint::Velocity(const unsigned int /*index*/) const
{
  gzlog << "Velocity not implemented for gearbox\n";
  return 0;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetVelocity(const unsigned int /*index*/,
    const double /*_angle*/)
{
  gzlog << "SetVelocity not implemented for gearbox\n";
  return;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetForceImpl(const unsigned int /*_index*/,
    const double /*_effort*/)
{
  if (this->odeJointDPtr->jointId)
    gzlog << "SetForceImpl not implemented for gearbox\n";
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEGearboxJoint::Param(const unsigned int /*_parameter*/) const
{
  gzlog << "Param not implemented for gearbox\n";
  return 0;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetParam(const unsigned int /*_parameter*/,
    const double /*_value*/)
{
  gzlog << "SetParam not implemented for gearbox\n";
  return;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEGearboxJoint::Anchor(
    const unsigned int /*_index*/) const
{
  dVector3 result;
  gzlog << "ODEGearboxJoint::Anchor not implemented.\n";
  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetAnchor(const unsigned int /*_index*/,
  const ignition::math::Vector3d &/*_anchor*/)
{
  gzlog << "ODEGearboxJoint::SetAnchor not implemented.\n";
}

//////////////////////////////////////////////////
double ODEGearboxJoint::Param(const std::string &_key,
                              const unsigned int _index) const
{
  if (_key  == "gearbox_raio")
    return this->GearboxRatio();
  else
    return ODEJoint::Param(_key, _index);
}
