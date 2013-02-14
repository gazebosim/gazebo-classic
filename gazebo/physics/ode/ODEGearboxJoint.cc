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
/* Desc: A ODEGearboxJoint
 * Author: Nate Koenig, Andrew Howard
 * Date: 21 May 2003
 */

#include <boost/bind.hpp>

#include "gazebo/gazebo_config.h"
#include "gazebo/common/Console.hh"

#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/ode/ODELink.hh"
#include "gazebo/physics/ode/ODEGearboxJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
ODEGearboxJoint::ODEGearboxJoint(dWorldID _worldId, BasePtr _parent)
    : GearboxJoint<ODEJoint>(_parent)
{
  this->jointId = dJointCreateGearbox(_worldId, NULL);
}

//////////////////////////////////////////////////
ODEGearboxJoint::~ODEGearboxJoint()
{
  if (this->applyDamping)
    physics::Joint::DisconnectJointUpdate(this->applyDamping);
}

void ODEGearboxJoint::Init()
{
  Joint::Init();
  LinkPtr link = this->model->GetLink(this->referenceBody);
  if (link)
    this->SetReferenceBody(link);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::Load(sdf::ElementPtr _sdf)
{
  GearboxJoint<ODEJoint>::Load(_sdf);

  this->SetGearRatio(this->gearRatio);
}

void ODEGearboxJoint::SetReferenceBody(LinkPtr _body)
{
  ODELinkPtr odelink = boost::shared_dynamic_cast<ODELink>(_body);

  if (odelink == NULL)
    gzwarn << "Reference body not valid, using inertial frame.\n";
  else
    dJointSetGearboxReferenceBody(this->jointId, odelink->GetODEId());
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetGearRatio(double _gearRatio)
{
  dJointSetGearboxRatio(this->jointId, _gearRatio);
}

//////////////////////////////////////////////////
math::Vector3 ODEGearboxJoint::GetAnchor(int _index) const
{
  dVector3 result;

  if (_index == 0)
    dJointGetGearboxAxis1(this->jointId, result);
  else if (_index == 1)
    dJointGetGearboxAxis2(this->jointId, result);
  else
    gzerr << "requesting GetAnchor axis [" << _index << "] out of range\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetAnchor(int _index, const math::Vector3 &_anchor)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  if (_index == 0)
    dJointSetGearboxAxis1(this->jointId, _anchor.x, _anchor.y, _anchor.z);
  else if (_index == 1)
    dJointSetGearboxAxis2(this->jointId, _anchor.x, _anchor.y, _anchor.z);
  else
    gzerr << "requesting SetAnchor axis [" << _index << "] out of range\n";
}


//////////////////////////////////////////////////
math::Vector3 ODEGearboxJoint::GetGlobalAxis(int _index) const
{
  dVector3 result;

  if (_index == 0)
    dJointGetGearboxAxis1(this->jointId, result);
  else if (_index == 1)
    dJointGetGearboxAxis2(this->jointId, result);
  else
    gzerr << "requesting GetAnchor axis [" << _index << "] out of range\n";

  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetAxis(int _index, const math::Vector3 &_axis)
{
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  if (_index == 0)
    dJointSetGearboxAxis1(this->jointId, _axis.x, _axis.y, _axis.z);
  else if (_index == 1)
    dJointSetGearboxAxis2(this->jointId, _axis.x, _axis.y, _axis.z);
  else
    gzerr << "requesting SetAnchor axis [" << _index << "] out of range\n";
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetDamping(int /*index*/, double _damping)
{
  gzlog << "damping not implemented for gearbox\n";
  return;

  this->dampingCoefficient = _damping;
  // use below when ode version is fixed
  // dJointSetDamping(this->jointId, this->dampingCoefficient);
  this->applyDamping = physics::Joint::ConnectJointUpdate(
    boost::bind(&Joint::ApplyDamping, this));
}

//////////////////////////////////////////////////
math::Angle ODEGearboxJoint::GetAngleImpl(int /*index*/) const
{
  gzlog << "GetAngle not implemented for gearbox\n";
  return math::Angle(0);

  math::Angle result;
  if (this->jointId)
    result = dJointGetHingeAngle(this->jointId);
  return result;
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetVelocity(int /*index*/) const
{
  gzlog << "GetVelocity not implemented for gearbox\n";
  return 0;

  double result = dJointGetHingeAngleRate(this->jointId);

  return result;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetVelocity(int /*index*/, double _angle)
{
  gzlog << "SetVelocity not implemented for gearbox\n";
  return;

  this->SetParam(dParamVel, _angle);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetMaxForce(int /*index*/, double _t)
{
  return this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetMaxForce(int /*index*/)
{
  return this->GetParam(dParamFMax);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetForce(int _index, double _torque)
{
  ODEJoint::SetForce(_index, _torque);
  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);
  dJointAddHingeTorque(this->jointId, _torque);
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetParam(int _parameter) const
{
  double result = dJointGetHingeParam(this->jointId, _parameter);

  return result;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetParam(int _parameter, double _value)
{
  ODEJoint::SetParam(_parameter, _value);

  dJointSetHingeParam(this->jointId, _parameter, _value);
}
