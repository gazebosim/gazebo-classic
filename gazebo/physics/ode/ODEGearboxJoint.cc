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

//////////////////////////////////////////////////
void ODEGearboxJoint::Init()
{
  Joint::Init();
  LinkPtr link = this->model->GetLink(this->referenceBody);
  if (link)
    this->SetReferenceBody(link);
  else
    gzerr << "reference Link has not been set yet.\n";
}

//////////////////////////////////////////////////
void ODEGearboxJoint::Load(sdf::ElementPtr _sdf)
{
  GearboxJoint<ODEJoint>::Load(_sdf);

  this->SetGearRatio(this->gearRatio);
}

//////////////////////////////////////////////////
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

  /// ODE needs global axis
  /// \TODO: currently we assume joint axis is specified in model frame,
  /// this is incorrect, and should be corrected to be
  /// joint frame which is specified in child link frame.
  math::Vector3 globalAxis = _axis;
  if (this->parentLink)
    globalAxis =
      this->GetParent()->GetModel()->GetWorldPose().rot.RotateVector(_axis);

  if (_index == 0)
    dJointSetGearboxAxis1(this->jointId, globalAxis.x, globalAxis.y, globalAxis.z);
  else if (_index == 1)
    dJointSetGearboxAxis2(this->jointId, globalAxis.x, globalAxis.y, globalAxis.z);
  else
    gzerr << "requesting SetAnchor axis [" << _index << "] out of range\n";
}

//////////////////////////////////////////////////
math::Angle ODEGearboxJoint::GetAngleImpl(int /*index*/) const
{
  gzlog << "GetAngle not implemented for gearbox\n";
  return math::Angle(0);

  math::Angle result;
  if (this->jointId)
    result = dJointGetHingeAngle(this->jointId);
  else
    gzerr << "ODE Joint ID is invalid\n";

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
  gzlog << "SetMaxForce not implemented for gearbox\n";
  return;

  return this->SetParam(dParamFMax, _t);
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetMaxForce(int /*index*/)
{
  gzlog << "GetMaxForce not implemented for gearbox\n";
  return 0;

  return this->GetParam(dParamFMax);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetForceImpl(int _index, double _effort)
{
  if (this->jointId)
    gzlog << "SetForce not implemented for gearbox\n";
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetParam(int _parameter) const
{
  gzlog << "GetParam not implemented for gearbox\n";
  return 0;

  double result = 0;

  if (this->jointId)
    result = dJointGetHingeParam(this->jointId, _parameter);
  else
    gzerr << "ODE Joint ID is invalid\n";

  return result;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetParam(int _parameter, double _value)
{
  gzlog << "SetParam not implemented for gearbox\n";
  return;

  ODEJoint::SetParam(_parameter, _value);
  if (this->jointId)
    dJointSetHingeParam(this->jointId, _parameter, _value);
  else
    gzerr << "ODE Joint ID is invalid\n";
}
