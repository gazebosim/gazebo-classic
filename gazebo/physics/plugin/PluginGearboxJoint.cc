/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/physics/PhysicsPlugin.h"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/plugin/PluginLink.hh"
#include "gazebo/physics/plugin/PluginGearboxJoint.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PluginGearboxJoint::PluginGearboxJoint(BasePtr _parent)
    : GearboxJoint<PluginJoint>(_parent)
{
  this->jointId = 0;
}

//////////////////////////////////////////////////
PluginGearboxJoint::~PluginGearboxJoint()
{
}

//////////////////////////////////////////////////
void PluginGearboxJoint::Init()
{
  Joint::Init();
  LinkPtr link = this->model->GetLink(this->referenceBody);
  this->SetReferenceBody(link);
}

//////////////////////////////////////////////////
void PluginGearboxJoint::Load(sdf::ElementPtr _sdf)
{
  GearboxJoint<PluginJoint>::Load(_sdf);

  this->SetGearboxRatio(this->gearRatio);
}

//////////////////////////////////////////////////
void PluginGearboxJoint::SetReferenceBody(LinkPtr _body)
{
}

//////////////////////////////////////////////////
void PluginGearboxJoint::SetGearboxRatio(double _gearRatio)
{
  this->gearRatio = _gearRatio;
}

//////////////////////////////////////////////////
math::Vector3 PluginGearboxJoint::GetGlobalAxis(unsigned int _index) const
{
  double result[3];
  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void PluginGearboxJoint::SetAxis(unsigned int _index, const math::Vector3 &_axis)
{
  PluginJoint::SetAxis(_index, _axis);

  if (this->childLink)
    this->childLink->SetEnabled(true);
  if (this->parentLink)
    this->parentLink->SetEnabled(true);

  /// Plugin needs global axis
  math::Quaternion axisFrame = this->GetAxisFrame(_index);
  math::Vector3 globalAxis = axisFrame.RotateVector(_axis);

}

//////////////////////////////////////////////////
math::Angle PluginGearboxJoint::GetAngleImpl(unsigned int /*index*/) const
{
  gzlog << "GetAngle not implemented for gearbox\n";
  return math::Angle(0);
}

//////////////////////////////////////////////////
double PluginGearboxJoint::GetVelocity(unsigned int /*index*/) const
{
  gzlog << "GetVelocity not implemented for gearbox\n";
  return 0;
}

//////////////////////////////////////////////////
void PluginGearboxJoint::SetVelocity(unsigned int /*index*/, double /*_angle*/)
{
  gzlog << "SetVelocity not implemented for gearbox\n";
  return;
}

//////////////////////////////////////////////////
void PluginGearboxJoint::SetMaxForce(unsigned int /*index*/, double /*_t*/)
{
  gzlog << "SetMaxForce not implemented for gearbox\n";
  return;
}

//////////////////////////////////////////////////
double PluginGearboxJoint::GetMaxForce(unsigned int /*index*/)
{
  gzlog << "GetMaxForce not implemented for gearbox\n";
  return 0;
}

//////////////////////////////////////////////////
void PluginGearboxJoint::SetForceImpl(unsigned int /*_index*/, double /*_effort*/)
{
  if (this->jointId)
    gzlog << "SetForce not implemented for gearbox\n";
  else
    gzerr << "Plugin Joint ID is invalid\n";
}

//////////////////////////////////////////////////
double PluginGearboxJoint::GetParam(unsigned int /*_parameter*/) const
{
  gzlog << "GetParam not implemented for gearbox\n";
  return 0;
}

//////////////////////////////////////////////////
void PluginGearboxJoint::SetParam(unsigned int /*_parameter*/, double /*_value*/)
{
  gzlog << "SetParam not implemented for gearbox\n";
  return;
}

//////////////////////////////////////////////////
math::Vector3 PluginGearboxJoint::GetAnchor(unsigned int /*_index*/) const
{
  double result[3];
  gzlog << "PluginGearboxJoint::GetAnchor not implemented.\n";
  return math::Vector3(result[0], result[1], result[2]);
}

//////////////////////////////////////////////////
void PluginGearboxJoint::SetAnchor(unsigned int /*_index*/,
  const math::Vector3 &/*_anchor*/)
{
  gzlog << "PluginGearboxJoint::SetAnchor not implemented.\n";
}
