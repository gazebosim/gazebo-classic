/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <ignition/math/Helpers.hh>

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
  this->jointId = dJointCreateGearbox(_worldId, nullptr);
}

//////////////////////////////////////////////////
ODEGearboxJoint::~ODEGearboxJoint()
{
}

//////////////////////////////////////////////////
void ODEGearboxJoint::Init()
{
  Joint::Init();
  LinkPtr link = this->model->GetLink(this->referenceBody);
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
  ODELinkPtr odelink = boost::dynamic_pointer_cast<ODELink>(_body);
  dBodyID refId;
  if (odelink == nullptr)
  {
    gzwarn << "Reference body not valid, using inertial frame.\n";
    refId = 0;
  }
  else
  {
    refId = odelink->GetODEId();
  }

  dJointSetGearboxReferenceBody(this->jointId, refId);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetGearboxRatio(double _gearRatio)
{
  this->gearRatio = _gearRatio;
  dJointSetGearboxRatio(this->jointId, _gearRatio);
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEGearboxJoint::GlobalAxis(
    const unsigned int _index) const
{
  dVector3 result;

  if (_index == 0)
    dJointGetGearboxAxis1(this->jointId, result);
  else if (_index == 1)
    dJointGetGearboxAxis2(this->jointId, result);
  else
  {
    gzerr << "index [" << _index << "] out of range\n";
    return ignition::math::Vector3d::Zero;
  }

  return ignition::math::Vector3d(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetAxis(const unsigned int _index,
    const ignition::math::Vector3d &_axis)
{
  ODEJoint::SetAxis(_index, _axis);

  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  /// ODE needs global axis
  auto axisFrame = this->AxisFrame(_index);
  auto globalAxis = axisFrame.RotateVector(_axis);

  if (_index == 0)
  {
    dJointSetGearboxAxis1(this->jointId, globalAxis.X(), globalAxis.Y(),
      globalAxis.Z());
  }
  else if (_index == 1)
  {
    dJointSetGearboxAxis2(this->jointId, globalAxis.X(), globalAxis.Y(),
      globalAxis.Z());
  }
  else
    gzerr << "index [" << _index << "] out of range\n";
}

//////////////////////////////////////////////////
double ODEGearboxJoint::PositionImpl(const unsigned int /*index*/) const
{
  gzlog << "GetAngle not implemented for gearbox\n";
  return ignition::math::NAN_D;
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetVelocity(unsigned int /*index*/) const
{
  gzlog << "GetVelocity not implemented for gearbox\n";
  return 0;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetVelocity(unsigned int /*index*/, double /*_angle*/)
{
  gzlog << "SetVelocity not implemented for gearbox\n";
  return;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  if (this->jointId)
    gzlog << "SetForce not implemented for gearbox\n";
  else
    gzerr << "ODE Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetParam(unsigned int /*_parameter*/) const
{
  gzlog << "GetParam not implemented for gearbox\n";
  return 0;
}

//////////////////////////////////////////////////
double ODEGearboxJoint::GetParam(const std::string &_key, unsigned int _index)
{
  if (_key  == "ratio")
    return dJointGetGearboxRatio(this->jointId);
  else if (_key  == "reference_angle1")
    return dJointGetGearboxReferenceAngle1(this->jointId);
  else if (_key  == "reference_angle2")
    return dJointGetGearboxReferenceAngle2(this->jointId);
  else
    return ODEJoint::GetParam(_key, _index);
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetParam(unsigned int /*_parameter*/, double /*_value*/)
{
  gzlog << "SetParam not implemented for gearbox\n";
  return;
}

//////////////////////////////////////////////////
bool ODEGearboxJoint::SetParam(const std::string &_key,
  unsigned int _index, const boost::any &_value)
{
  try
  {
    if (_key  == "ratio")
    {
      this->SetGearboxRatio(boost::any_cast<double>(_value));
    }
    else if (_key  == "reference_angle1")
    {
      dJointSetGearboxReferenceAngle1(this->jointId,
          boost::any_cast<double>(_value));
    }
    else if (_key  == "reference_angle2")
    {
      dJointSetGearboxReferenceAngle2(this->jointId,
          boost::any_cast<double>(_value));
    }
    else
    {
      return ODEJoint::SetParam(_key, _index, _value);
    }
  }
  catch(const boost::bad_any_cast &e)
  {
    gzerr << "boost any_cast error:" << e.what() << "\n";
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
ignition::math::Vector3d ODEGearboxJoint::Anchor(
    const unsigned int /*_index*/) const
{
  gzlog << "ODEGearboxJoint::Anchor not implemented.\n";
  return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
void ODEGearboxJoint::SetAnchor(const unsigned int /*_index*/,
  const ignition::math::Vector3d &/*_anchor*/)
{
  gzlog << "ODEGearboxJoint::SetAnchor not implemented.\n";
}
